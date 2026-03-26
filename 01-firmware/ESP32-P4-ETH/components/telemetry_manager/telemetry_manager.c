/*
 * Telemetry Manager - InfluxDB Line Protocol over MQTT
 * Optimized version: No dynamic allocations during field addition/commit.
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
#include "ptp_client.h"
#include "telemetry_manager.h"

// =============================================================================
// Definitions & Internal Types
// =============================================================================

static const char *TAG = "telemetry";

// Reduced from 32KB to 4KB (standard for MQTT payloads and saves internal RAM)
#define MAX_BUFFER_SIZE 4096

typedef struct {
    char *topic;
    char *measurement;
    char *tags;
    uint32_t interval_ms;
    
    // Internal State
    char *batch_buffer;
    int batch_offset;
    SemaphoreHandle_t mutex;
    TaskHandle_t task_handle;
    bool running;
} telemetry_obj_t;

// =============================================================================
// Internal Handlers & Tasks
// =============================================================================

// Internal task function remains basically the same
static void telemetry_task(void *arg)
{
    telemetry_obj_t *obj = (telemetry_obj_t *)arg;
    TickType_t period = pdMS_TO_TICKS(obj->interval_ms);
    
    while (obj->running) {
        vTaskDelay(period);

        if (!mqtt_custom_client_is_connected()) {
            continue;
        }

        if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (obj->batch_offset > 0) {
                // Publish the accumulated batch buffer
                mqtt_custom_client_publish(obj->topic, obj->batch_buffer, 0, 0, 0);
                
                // Clear batch buffer
                obj->batch_offset = 0;
                obj->batch_buffer[0] = '\0';
            }
            xSemaphoreGive(obj->mutex);
        }
    }

    vTaskDelete(NULL);
}

// =============================================================================
// Public API: Registry & Lifecycle
// =============================================================================

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

    char task_name[16];
    snprintf(task_name, sizeof(task_name), "tel_%s", measurement);
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
    
    if (obj->mutex) vSemaphoreDelete(obj->mutex);
    free(obj);
}

void telemetry_set_tags(telemetry_handle_t handle, const char *tags)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (obj->tags) free(obj->tags);
        obj->tags = tags ? strdup(tags) : NULL;
        xSemaphoreGive(obj->mutex);
    }
}

// =============================================================================
// Optimized Implementation: Direct Append
// =============================================================================

static void append_to_batch(telemetry_obj_t *obj, const char *key, const char *val_str) {
    if (!obj || !key || !val_str) return;

    int64_t timestamp_ns = get_ptp_timestamp_us() * 1000ULL;

#ifdef CONFIG_TELEMETRY_ROBOT_NAME
    const char *global_robot_tag = "robot=" CONFIG_TELEMETRY_ROBOT_NAME;
#else
    const char *global_robot_tag = "robot=unknown";
#endif

    char line_buf[256];
    int len = 0;

    // Measurement[,tags] field=val timestamp\n
    if (obj->tags && strlen(obj->tags) > 0) {
        len = snprintf(line_buf, sizeof(line_buf), "%s,%s,%s %s=%s %lld\n", 
                        obj->measurement, global_robot_tag, obj->tags,
                        key, val_str, timestamp_ns);
    } else {
        len = snprintf(line_buf, sizeof(line_buf), "%s,%s %s=%s %lld\n", 
                        obj->measurement, global_robot_tag,
                        key, val_str, timestamp_ns);
    }

    if (len > 0 && (obj->batch_offset + len < MAX_BUFFER_SIZE - 1)) {
        memcpy(obj->batch_buffer + obj->batch_offset, line_buf, len);
        obj->batch_offset += len;
        obj->batch_buffer[obj->batch_offset] = '\0';
    } else {
        ESP_LOGW(TAG, "Batch buffer full or format fail for %s", obj->measurement);
    }
}

void telemetry_add_float(telemetry_handle_t handle, const char *key, float value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    char val_str[24];
    snprintf(val_str, sizeof(val_str), "%.3f", value);
    
    if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        append_to_batch(obj, key, val_str);
        xSemaphoreGive(obj->mutex);
    }
}

void telemetry_add_int(telemetry_handle_t handle, const char *key, int32_t value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    char val_str[16];
    snprintf(val_str, sizeof(val_str), "%li", (long)value);
    
    if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        append_to_batch(obj, key, val_str);
        xSemaphoreGive(obj->mutex);
    }
}

void telemetry_add_bool(telemetry_handle_t handle, const char *key, bool value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        append_to_batch(obj, key, value ? "true" : "false");
        xSemaphoreGive(obj->mutex);
    }
}

/**
 * commit_point is now a no-op as telemetry_add_* appends directly.
 * Kept for API compatibility.
 */
void telemetry_commit_point(telemetry_handle_t handle)
{
    (void)handle;
}
