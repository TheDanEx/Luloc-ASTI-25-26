#include "shared_memory.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "SHARED_MEM";

// Internal Queues
static QueueHandle_t q_telemetry_fast = NULL;
static QueueHandle_t q_telemetry_slow = NULL;
static QueueHandle_t q_mode_cmd = NULL;
static QueueHandle_t q_twist_cmd = NULL;

esp_err_t shared_memory_init(void) {
    q_telemetry_fast = xQueueCreate(1, sizeof(robot_telemetry_fast_t));
    q_telemetry_slow = xQueueCreate(1, sizeof(robot_telemetry_slow_t));
    q_mode_cmd = xQueueCreate(5, sizeof(robot_mode_cmd_t));
    q_twist_cmd = xQueueCreate(5, sizeof(robot_twist_cmd_t));

    if (!q_telemetry_fast || !q_telemetry_slow || !q_mode_cmd || !q_twist_cmd) {
        ESP_LOGE(TAG, "Failed to create IPC queues");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "IPC Shared Memory initialized (Grouped Mode)");
    return ESP_OK;
}

// =============================================================================
// Implementation Macros for Boilerplate
// =============================================================================

#define PUSH_OVERWRITE(queue, data_ptr) \
    (xQueueOverwrite(queue, data_ptr) == pdPASS ? ESP_OK : ESP_FAIL)

#define GET_NONBLOCKING(queue, data_ptr) \
    (xQueueReceive(queue, data_ptr, 0) == pdPASS ? ESP_OK : ESP_ERR_NOT_FOUND)

#define PUT_FIFO(queue, data_ptr) \
    (xQueueSend(queue, data_ptr, 0) == pdPASS ? ESP_OK : ESP_FAIL)

// =============================================================================
// API Functions
// =============================================================================

esp_err_t shared_memory_push_telemetry_fast(const robot_telemetry_fast_t *data) { return PUSH_OVERWRITE(q_telemetry_fast, data); }
esp_err_t shared_memory_push_telemetry_slow(const robot_telemetry_slow_t *data) { return PUSH_OVERWRITE(q_telemetry_slow, data); }

esp_err_t shared_memory_get_mode_cmd(robot_mode_cmd_t *cmd) { return GET_NONBLOCKING(q_mode_cmd, cmd); }
esp_err_t shared_memory_get_twist_cmd(robot_twist_cmd_t *cmd) { return GET_NONBLOCKING(q_twist_cmd, cmd); }

esp_err_t shared_memory_get_telemetry_fast(robot_telemetry_fast_t *data) { return GET_NONBLOCKING(q_telemetry_fast, data); }
esp_err_t shared_memory_get_telemetry_slow(robot_telemetry_slow_t *data) { return GET_NONBLOCKING(q_telemetry_slow, data); }

esp_err_t shared_memory_put_mode_cmd(const robot_mode_cmd_t *cmd) { return PUT_FIFO(q_mode_cmd, cmd); }
esp_err_t shared_memory_put_twist_cmd(const robot_twist_cmd_t *cmd) { return PUT_FIFO(q_twist_cmd, cmd); }
