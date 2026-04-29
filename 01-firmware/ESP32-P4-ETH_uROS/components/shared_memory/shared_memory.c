#include "shared_memory.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "SHARED_MEM";

// =============================================================================
// Static Variables (Private)
// =============================================================================

static QueueHandle_t line_sensors_queue = NULL;
static QueueHandle_t calibration_queue = NULL;
static QueueHandle_t pid_data_queue = NULL;
static QueueHandle_t odometry_queue = NULL;
static QueueHandle_t state_queue = NULL;
static QueueHandle_t mode_cmd_queue = NULL;
static QueueHandle_t twist_cmd_queue = NULL;

// =============================================================================
// Public API Implementation
// =============================================================================

esp_err_t shared_memory_init(void) {
    // 1. Core 1 -> Core 0 (Telemetry & State): Overwrite Queues (Size 1)
    line_sensors_queue = xQueueCreate(1, sizeof(robot_line_sensors_t));
    calibration_queue = xQueueCreate(1, sizeof(robot_calibration_t));
    pid_data_queue = xQueueCreate(1, sizeof(robot_pid_data_t));
    odometry_queue = xQueueCreate(1, sizeof(robot_odometry_t));
    state_queue = xQueueCreate(1, sizeof(robot_state_t));

    // 2. Core 0 -> Core 1 (Commands): FIFO Queues
    mode_cmd_queue = xQueueCreate(5, sizeof(robot_mode_cmd_t));
    twist_cmd_queue = xQueueCreate(5, sizeof(robot_twist_cmd_t));

    if (!line_sensors_queue || !calibration_queue || !pid_data_queue || 
        !odometry_queue || !state_queue || !mode_cmd_queue || !twist_cmd_queue) {
        ESP_LOGE(TAG, "Failed to create IPC queues");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "IPC Shared Memory initialized");
    return ESP_OK;
}

// Helper macro for overwrite push
#define PUSH_OVERWRITE(queue, data) if (queue != NULL) xQueueOverwrite(queue, data)

// Helper macro for non-blocking pop
#define GET_NON_BLOCK(queue, data) (queue != NULL && xQueueReceive(queue, data, 0) == pdTRUE) ? ESP_OK : ESP_ERR_NOT_FOUND

// --- Line Sensors ---
void shared_memory_push_line_sensors(const robot_line_sensors_t *data) { PUSH_OVERWRITE(line_sensors_queue, data); }
esp_err_t shared_memory_get_line_sensors(robot_line_sensors_t *data) { return GET_NON_BLOCK(line_sensors_queue, data); }

// --- Calibration ---
void shared_memory_push_calibration(const robot_calibration_t *data) { PUSH_OVERWRITE(calibration_queue, data); }
esp_err_t shared_memory_get_calibration(robot_calibration_t *data) { return GET_NON_BLOCK(calibration_queue, data); }

// --- PID Data ---
void shared_memory_push_pid_data(const robot_pid_data_t *data) { PUSH_OVERWRITE(pid_data_queue, data); }
esp_err_t shared_memory_get_pid_data(robot_pid_data_t *data) { return GET_NON_BLOCK(pid_data_queue, data); }

// --- Odometry ---
void shared_memory_push_odometry(const robot_odometry_t *data) { PUSH_OVERWRITE(odometry_queue, data); }
esp_err_t shared_memory_get_odometry(robot_odometry_t *data) { return GET_NON_BLOCK(odometry_queue, data); }

// --- State ---
void shared_memory_push_state(const robot_state_t *data) { PUSH_OVERWRITE(state_queue, data); }
esp_err_t shared_memory_get_state(robot_state_t *data) { return GET_NON_BLOCK(state_queue, data); }

// --- Commands (Core 0 -> Core 1) ---

void shared_memory_push_mode_cmd(const robot_mode_cmd_t *cmd) {
    if (mode_cmd_queue != NULL) xQueueSend(mode_cmd_queue, cmd, 0);
}

esp_err_t shared_memory_get_mode_cmd(robot_mode_cmd_t *cmd) { return GET_NON_BLOCK(mode_cmd_queue, cmd); }

void shared_memory_push_twist_cmd(const robot_twist_cmd_t *cmd) {
    if (twist_cmd_queue != NULL) xQueueSend(twist_cmd_queue, cmd, 0);
}

esp_err_t shared_memory_get_twist_cmd(robot_twist_cmd_t *cmd) { return GET_NON_BLOCK(twist_cmd_queue, cmd); }
