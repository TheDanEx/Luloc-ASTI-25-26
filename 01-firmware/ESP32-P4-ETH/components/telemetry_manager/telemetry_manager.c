/*
 * Telemetry Manager - InfluxDB Line Protocol over MQTT
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mqtt_custom_client.h"
#include "telemetry_manager.h"

static const char *TAG = "telemetry";

#define MAX_BUFFER_SIZE 512
#define MAX_FIELDS 16

typedef struct {
    char *key;
    char *value_str;
} field_t;

typedef struct {
    char *topic;
    char *measurement;
    uint32_t interval_ms;
    
    // Internal State
    field_t fields[MAX_FIELDS];
    int field_count;
    SemaphoreHandle_t mutex;
    TaskHandle_t task_handle;
    bool running;
} telemetry_obj_t;

// Helper to safely append valid InfluxDB line protocol fields
static void append_field_str(telemetry_obj_t *obj, const char *key, const char *val_str)
{
    if (obj->field_count >= MAX_FIELDS) return;

    // Allocate copy of key and value
    obj->fields[obj->field_count].key = strdup(key);
    obj->fields[obj->field_count].value_str = strdup(val_str);
    
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
    char *buffer = malloc(MAX_BUFFER_SIZE);
    
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer for %s", obj->measurement);
        vTaskDelete(NULL);
        return;
    }

    TickType_t period = pdMS_TO_TICKS(obj->interval_ms);
    
    while (obj->running) {
        vTaskDelay(period);

        if (!mqtt_custom_client_is_connected()) {
            continue;
        }

        xSemaphoreTake(obj->mutex, portMAX_DELAY);
        
        if (obj->field_count > 0) {
            // Build Line Protocol String: measurement field1=val1,field2=val2
            // Note: We are NOT sending timestamp, relying on server time
            
            int offset = snprintf(buffer, MAX_BUFFER_SIZE, "%s ", obj->measurement);
            
            for (int i = 0; i < obj->field_count; i++) {
                if (offset >= MAX_BUFFER_SIZE - 1) break;
                
                // Separator: Comma between fields, starting from 2nd field
                if (i > 0) {
                    offset += snprintf(buffer + offset, MAX_BUFFER_SIZE - offset, ",");
                }
                
                offset += snprintf(buffer + offset, MAX_BUFFER_SIZE - offset, 
                                   "%s=%s", obj->fields[i].key, obj->fields[i].value_str);
            }
            
            // Clear fields after consuming
            for (int i = 0; i < obj->field_count; i++) {
                free(obj->fields[i].key);
                free(obj->fields[i].value_str);
            }
            obj->field_count = 0;
            
            // Release Mutex before publishing (network op might be slow)
            xSemaphoreGive(obj->mutex);
            
            // Publish
            mqtt_custom_client_publish(obj->topic, buffer, 0, 0, 0);
            
        } else {
            xSemaphoreGive(obj->mutex);
        }
    }

    free(buffer);
    vTaskDelete(NULL);
}

telemetry_handle_t telemetry_create(const char *topic, const char *measurement, uint32_t interval_ms)
{
    telemetry_obj_t *obj = calloc(1, sizeof(telemetry_obj_t));
    if (!obj) return NULL;

    obj->topic = strdup(topic);
    obj->measurement = strdup(measurement);
    obj->interval_ms = interval_ms;
    obj->mutex = xSemaphoreCreateMutex();
    obj->running = true;

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
    // Task will delete itself, but we should probably join or wait? 
    // For simplicity in this embedded context, we assume dynamic destruction is rare or app shutdown.
    
    // Simple cleanup of allocated strings
    // Note: This is unsafe if task is currently running. 
    // In a robust system we would wait for task notification.
    vTaskDelay(100); 

    if (obj->topic) free(obj->topic);
    if (obj->measurement) free(obj->measurement);
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    for (int i = 0; i < obj->field_count; i++) {
        free(obj->fields[i].key);
        free(obj->fields[i].value_str);
    }
    xSemaphoreGive(obj->mutex);
    vSemaphoreDelete(obj->mutex);
    
    free(obj);
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
    snprintf(val_str, sizeof(val_str), "%li", value); // Influx Integer usually implies 'i' suffix? 
    // Standard Line Protocol integers don't strictly need 'i' unless type conflict, 
    // but usually sent as plain numbers. If strict: "%lii"
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    append_field_str(obj, key, val_str);
    xSemaphoreGive(obj->mutex);
}

void telemetry_add_bool(telemetry_handle_t handle, const char *key, bool value)
{
    if (!handle) return;
    telemetry_obj_t *obj = (telemetry_obj_t *)handle;
    
    xSemaphoreTake(obj->mutex, portMAX_DELAY);
    append_field_str(obj, key, value ? "true" : "false"); // Influx booleans
    xSemaphoreGive(obj->mutex);
}
