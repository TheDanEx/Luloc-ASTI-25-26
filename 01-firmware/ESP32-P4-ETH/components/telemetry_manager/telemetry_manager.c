/*
 * Telemetry Manager - InfluxDB Line Protocol over MQTT
 * SPDX-License-Identifier: MIT
 */

#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <time.h>
#include "mqtt_custom_client.h"
#include "telemetry_manager.h"

// =============================================================================
// Definitions & Internal Types
// =============================================================================

static const char *TAG = "telemetry";

#define MAX_FIELDS 16
#define MAX_BUFFER_SIZE 4096

typedef struct {
    char *key;
    char *value_str;
    int64_t timestamp_ns;
} field_t;

typedef struct {
    char *topic;
    char *measurement;
    char *tags;
    uint32_t interval_ms;
    
    // Internal State
    field_t fields[MAX_FIELDS];
    int field_count;
    char *batch_buffer;
    int batch_offset;
    SemaphoreHandle_t mutex;
    TaskHandle_t task_handle;
    bool running;
} telemetry_obj_t;

// =============================================================================
// Internal Handlers & Tasks
// =============================================================================

/**
 * Helper to safely append valid InfluxDB line protocol fields to the internal list
 */
static void append_field_str(telemetry_obj_t *obj, const char *key, const char *val_str)
{
    if (obj->field_count >= MAX_FIELDS) return;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    int64_t timestamp_ns = (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;

    // Allocate copy of key and value
    obj->fields[obj->field_count].key = strdup(key);
    obj->fields[obj->field_count].value_str = strdup(val_str);
    obj->fields[obj->field_count].timestamp_ns = timestamp_ns;
    
    if (obj->fields[obj->field_count].key && obj->fields[obj->field_count].value_str) {
        obj->field_count++;
    } else {
        // Cleanup if allocation failed
        if (obj->fields[obj->field_count].key) free(obj->fields[obj->field_count].key);
        if (obj->fields[obj->field_count].value_str) free(obj->fields[obj->field_count].value_str);
    }
}

// Internal task function
static void telemetry_task(void *arg)
{
    telemetry_obj_t *obj = (telemetry_obj_t *)arg;
    
    TickType_t period = pdMS_TO_TICKS(obj->interval_ms);
    
    while (obj->running) {
        vTaskDelay(period);

        if (!mqtt_custom_client_is_connected()) {
            continue;
        }

        xSemaphoreTake(obj->mutex, portMAX_DELAY);
        
        if (obj->batch_offset > 0) {
            // Publish the accumulated batch buffer
            mqtt_custom_client_publish(obj->topic, obj->batch_buffer, 0, 0, 0);
            
            // Clear batch buffer
            obj->batch_offset = 0;
            obj->batch_buffer[0] = '\0';
        }
        
        xSemaphoreGive(obj->mutex);
    }

    vTaskDelete(NULL);
}

// =============================================================================
// Public API: Registry & Lifecycle
// =============================================================================

/**
 * Create a new telemetry reporter mapped to a specific topic
 */
telemetry_handle_t telemetry_create(const char *topic, const char *measurement, uint32_t interval_ms)
{
    telemetry_obj_t *obj = calloc(1, sizeof(telemetry_obj_t));
    if (!obj) return NULL;

    obj->topic = strdup(topic);
    obj->measurement = strdup(measurement);
    obj->interval_ms = interval_ms;
    obj->mutex = xSemaphoreCreateMutex();
    obj->running = true;

    obj->batch_buffer = calloc(1, MAX_BUFFER_SIZE);
    if (!obj->batch_buffer) {
        telemetry_destroy(obj);
        return NULL;
    }

    // Create task pinned to CPU1 as requested
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "tel_%s", measurement);
    // Limit name length for FreeRTOS
    task_name[configMAX_TASK_NAME_LEN - 1] = '\0';

    BaseType_t res = xTaskCreatePinnedToCore(telemetry_task, task_name, 3072, obj, 4, &obj->task_handle, 1);
    
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task for %s", measurement);
        telemetry_destroy(obj);
        return NULL;
    }

    ESP_LOGI(TAG, "Telemetry created: %s -> %s (%lu ms)", measurement, topic, interval_ms);
    return (telemetry_handle_t)obj;
}

void telemetry_destroy(telemetry_handle_t handle)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;

    obj->running = false;
    vTaskDelay(pdMS_TO_TICKS(100)); 

    if (obj->topic) free(obj->topic);
    if (obj->measurement) free(obj->measurement);
    if (obj->tags) free(obj->tags);
    if (obj->batch_buffer) free(obj->batch_buffer);
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    for (int i = 0; i < obj->field_count; i++) {
        free(obj->fields[i].key);
        free(obj->fields[i].value_str);
    }
    xSemaphoreGive(obj->mutex);
    vSemaphoreDelete(obj->mutex);
    
    free(obj);
}

void telemetry_set_tags(telemetry_handle_t handle, const char *tags)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    if (obj->tags) free(obj->tags);
    obj->tags = tags ? strdup(tags) : NULL;
    xSemaphoreGive(obj->mutex);
}

// =============================================================================
// Public API: Field Management & Commit
// =============================================================================

/**
 * Format the collected fields into the batch buffer and clear the current point
 */
void telemetry_commit_point(telemetry_handle_t handle)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    
    if (obj->field_count > 0) {
        // Format: measurement[,tags] field=val timestamp\n
        // Multi-line ILP to support independent field timestamps.
        
#ifdef CONFIG_TELEMETRY_ROBOT_NAME
        const char *global_robot_tag = "robot=" CONFIG_TELEMETRY_ROBOT_NAME;
#else
        const char *global_robot_tag = "robot=unknown";
#endif

        for (int i = 0; i < obj->field_count; i++) {
            char line_buf[256];
            int len = 0;

            if (obj->tags && strlen(obj->tags) > 0) {
                len = snprintf(line_buf, sizeof(line_buf), "%s,%s,%s %s=%s %lld\n", 
                              obj->measurement, global_robot_tag, obj->tags,
                              obj->fields[i].key, obj->fields[i].value_str,
                              obj->fields[i].timestamp_ns);
            } else {
                len = snprintf(line_buf, sizeof(line_buf), "%s,%s %s=%s %lld\n", 
                              obj->measurement, global_robot_tag,
                              obj->fields[i].key, obj->fields[i].value_str,
                              obj->fields[i].timestamp_ns);
            }

            // Append to batch buffer if there's space
            if (obj->batch_offset + len < MAX_BUFFER_SIZE - 1) {
                memcpy(obj->batch_buffer + obj->batch_offset, line_buf, len);
                obj->batch_offset += len;
                obj->batch_buffer[obj->batch_offset] = '\0';
            }

            // Free the memory for the field strings
            free(obj->fields[i].key);
            free(obj->fields[i].value_str);
        }
        
        obj->field_count = 0;
    }
    
    xSemaphoreGive(obj->mutex);
}

void telemetry_add_float(telemetry_handle_t handle, const char *key, float value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    char val_str[32];
    snprintf(val_str, sizeof(val_str), "%.3f", value);
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    append_field_str(obj, key, val_str);
    xSemaphoreGive(obj->mutex);
}

void telemetry_add_int(telemetry_handle_t handle, const char *key, int32_t value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    char val_str[32];
    snprintf(val_str, sizeof(val_str), "%d", (int)value); 
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    append_field_str(obj, key, val_str);
    xSemaphoreGive(obj->mutex);
}

void telemetry_add_bool(telemetry_handle_t handle, const char *key, bool value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    append_field_str(obj, key, value ? "true" : "false");
    xSemaphoreGive(obj->mutex);
}
