/*
 * Telemetry Manager - Optimized Hierarchical Double-Buffered Implementation
 * Optimized for ESP32-P4 to fit in static BSS while providing 500Hz capability.
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
#include "mqtt_custom_client.h"
#include "ptp_client.h"
#include "telemetry_manager.h"

// =============================================================================
// Static Pool Allocation (Hierarchical)
// =============================================================================

static const char *TAG = "telemetry";

#define BUF_SIZE_HIGH 65536 // 64KB for Instance 0 (High Frequency: Siguelineas)
#define BUF_SIZE_LOW  8192  // 8KB for Instances 1 & 2 (Low Frequency: Odom/System)

typedef struct {
    char topic[64];
    char measurement[32];
    char tags[128];
    uint32_t interval_ms;
    
    // Pointers to the static buffers
    char *buf_a;
    char *buf_b;
    uint32_t max_buf_size;
    
    char *active_buf;
    int active_offset;
    
    SemaphoreHandle_t mutex;
    TaskHandle_t task_handle;
    bool running;
    bool in_use;
} telemetry_obj_internal_t;

// Reserve memories statically to avoid heap fragmentation and linker overflows
static char s_buf_inst0_a[BUF_SIZE_HIGH];
static char s_buf_inst0_b[BUF_SIZE_HIGH];
static char s_buf_inst1_a[BUF_SIZE_LOW];
static char s_buf_inst1_b[BUF_SIZE_LOW];
static char s_buf_inst2_a[BUF_SIZE_LOW];
static char s_buf_inst2_b[BUF_SIZE_LOW];

static telemetry_obj_internal_t s_pool[3] = {
    { .buf_a = s_buf_inst0_a, .buf_b = s_buf_inst0_b, .max_buf_size = BUF_SIZE_HIGH, .in_use = false },
    { .buf_a = s_buf_inst1_a, .buf_b = s_buf_inst1_b, .max_buf_size = BUF_SIZE_LOW,  .in_use = false },
    { .buf_a = s_buf_inst2_a, .buf_b = s_buf_inst2_b, .max_buf_size = BUF_SIZE_LOW,  .in_use = false }
};

// =============================================================================
// Internal Task
// =============================================================================

static void telemetry_task(void *arg)
{
    telemetry_obj_internal_t *obj = (telemetry_obj_internal_t *)arg;
    TickType_t period = pdMS_TO_TICKS(obj->interval_ms);
    
    while (obj->running) {
        vTaskDelay(period);

        if (!mqtt_custom_client_is_connected()) {
            continue;
        }

        int size_to_send = 0;
        char *buffer_to_send = NULL;

        if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (obj->active_offset > 0) {
                buffer_to_send = obj->active_buf;
                size_to_send = obj->active_offset;

                // Ping-Pong swap
                obj->active_buf = (obj->active_buf == obj->buf_a) ? obj->buf_b : obj->buf_a;
                obj->active_offset = 0;
                obj->active_buf[0] = '\0';
            }
            xSemaphoreGive(obj->mutex);
        }

        if (buffer_to_send && size_to_send > 0) {
            mqtt_custom_client_publish(obj->topic, buffer_to_send, size_to_send, 0, 0);
        }
    }
    vTaskDelete(NULL);
}

// =============================================================================
// Public API
// =============================================================================

telemetry_handle_t telemetry_create(const char *topic, const char *measurement, uint32_t interval_ms)
{
    telemetry_obj_internal_t *obj = NULL;
    
    // Find free slot
    for (int i = 0; i < 3; i++) {
        if (!s_pool[i].in_use) {
            obj = &s_pool[i];
            obj->in_use = true;
            break;
        }
    }

    if (!obj) {
        ESP_LOGE(TAG, "No free telemetry slots");
        return NULL;
    }

    strncpy(obj->topic, topic, sizeof(obj->topic) - 1);
    strncpy(obj->measurement, measurement, sizeof(obj->measurement) - 1);
    obj->interval_ms = interval_ms;
    obj->mutex = xSemaphoreCreateMutex();
    obj->running = true;
    obj->active_buf = obj->buf_a;
    obj->active_offset = 0;
    obj->active_buf[0] = '\0';

    char task_name[16];
    snprintf(task_name, sizeof(task_name), "tel_%s", measurement);
    task_name[configMAX_TASK_NAME_LEN - 1] = '\0';

    xTaskCreatePinnedToCore(telemetry_task, task_name, 4096, obj, 4, &obj->task_handle, 1);
    
    ESP_LOGI(TAG, "Telemetry created: %s, Buffer size: %lu KB", measurement, obj->max_buf_size/1024);
    return (telemetry_handle_t)obj;
}

void telemetry_destroy(telemetry_handle_t handle)
{
    if (!handle) return;
    telemetry_obj_internal_t *obj = (telemetry_obj_internal_t *)handle;
    obj->running = false;
    vTaskDelay(pdMS_TO_TICKS(100)); 
    if (obj->mutex) vSemaphoreDelete(obj->mutex);
    obj->in_use = false;
}

void telemetry_set_tags(telemetry_handle_t handle, const char *tags)
{
    if (!handle) return;
    telemetry_obj_internal_t *obj = (telemetry_obj_internal_t *)handle;
    if (xSemaphoreTake(obj->mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (tags) strncpy(obj->tags, tags, sizeof(obj->tags) - 1);
        else obj->tags[0] = '\0';
        xSemaphoreGive(obj->mutex);
    }
}

static void append_to_batch(telemetry_obj_internal_t *obj, const char *key, const char *val_str) {
    int64_t timestamp_ns = get_ptp_timestamp_us() * 1000ULL;
#ifdef CONFIG_TELEMETRY_ROBOT_NAME
    const char *global_robot_tag = "robot=" CONFIG_TELEMETRY_ROBOT_NAME;
#else
    const char *global_robot_tag = "robot=unknown";
#endif

    char line_buf[256];
    int len = 0;
    if (obj->tags[0] != '\0') {
        len = snprintf(line_buf, sizeof(line_buf), "%s,%s,%s %s=%s %lld\n", 
                        obj->measurement, global_robot_tag, obj->tags,
                        key, val_str, timestamp_ns);
    } else {
        len = snprintf(line_buf, sizeof(line_buf), "%s,%s %s=%s %lld\n", 
                        obj->measurement, global_robot_tag,
                        key, val_str, timestamp_ns);
    }

    if (len > 0 && (obj->active_offset + len < obj->max_buf_size - 1)) {
        memcpy(obj->active_buf + obj->active_offset, line_buf, len);
        obj->active_offset += len;
        obj->active_buf[obj->active_offset] = '\0';
    } else if (len > 0) {
        static uint32_t last_warn = 0;
        uint32_t now = esp_timer_get_time() / 1000000;
        if (now - last_warn > 5) {
            ESP_LOGW(TAG, "Buffer FULL for %s", obj->measurement);
            last_warn = now;
        }
    }
}

void telemetry_add_float(telemetry_handle_t handle, const char *key, float value)
{
    if (!handle) return;
    telemetry_obj_internal_t *obj = (telemetry_obj_internal_t *)handle;
    char val_str[24];
    snprintf(val_str, sizeof(val_str), "%.3f", value);
    if (xSemaphoreTake(obj->mutex, 0) == pdTRUE) {
        append_to_batch(obj, key, val_str);
        xSemaphoreGive(obj->mutex);
    }
}

void telemetry_add_int(telemetry_handle_t handle, const char *key, int32_t value)
{
    if (!handle) return;
    telemetry_obj_internal_t *obj = (telemetry_obj_internal_t *)handle;
    char val_str[16];
    snprintf(val_str, sizeof(val_str), "%li", (long)value);
    if (xSemaphoreTake(obj->mutex, 0) == pdTRUE) {
        append_to_batch(obj, key, val_str);
        xSemaphoreGive(obj->mutex);
    }
}

void telemetry_add_bool(telemetry_handle_t handle, const char *key, bool value)
{
    if (!handle) return;
    telemetry_obj_internal_t *obj = (telemetry_obj_internal_t *)handle;
    if (xSemaphoreTake(obj->mutex, 0) == pdTRUE) {
        append_to_batch(obj, key, value ? "true" : "false");
        xSemaphoreGive(obj->mutex);
    }
}

void telemetry_commit_point(telemetry_handle_t handle)
{
    (void)handle;
}
