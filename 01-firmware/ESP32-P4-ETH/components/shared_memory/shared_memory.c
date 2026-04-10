#include "shared_memory.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "shrd_mem";

// Global shared memory instance
static shared_memory_t g_shared_memory = {0};
static bool g_initialized = false;

// =============================================================================
// Public API: Lifecycle
// =============================================================================

/**
 * Initialize the shared memory singleton.
 * Configures the FreeRTOS mutex for thread-safe access between CPU cores.
 */
void shared_memory_init(void)
{
    if (g_initialized) {
        return;
    }

    g_shared_memory.mutex = xSemaphoreCreateMutex();
    if (g_shared_memory.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Default zero-state
    g_shared_memory.sensors = (robot_sensor_data_t){0};
    g_shared_memory.last_command = (robot_command_t){0};
    g_shared_memory.heartbeat_cpu0 = 0;
    g_shared_memory.heartbeat_cpu1 = 0;
    g_shared_memory.cpu0_alive = false;
    g_shared_memory.cpu1_alive = false;
    g_shared_memory.calibration_motor_mask = 3; 

    // Initialize Line PID parameters from Kconfig defaults
    g_shared_memory.line_pid.kp = atof(CONFIG_FOLLOW_LINE_KP);
    g_shared_memory.line_pid.ki = atof(CONFIG_FOLLOW_LINE_KI);
    g_shared_memory.line_pid.kd = atof(CONFIG_FOLLOW_LINE_KD);
    g_shared_memory.line_pid.updated_flag = false;

    // Initialize Line Sensor Calibration bounds
    for (int i = 0; i < 8; i++) {
        g_shared_memory.sensors.line_min[i] = 4095;
        g_shared_memory.sensors.line_max[i] = 0;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "Shared memory initialized");
}

// =============================================================================
// Public API: Sensor Data (CPU0 -> CPU1)
// =============================================================================

/**
 * Update the global sensor snapshot.
 * Called by CPU0 after a control loop iteration.
 */
bool shared_memory_write_sensors(const robot_sensor_data_t *data, TickType_t timeout)
{
    if (!g_initialized || data == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, timeout) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for sensor write (timeout)");
        return false;
    }

    g_shared_memory.sensors = *data;
    g_shared_memory.cpu0_alive = true;
    
    xSemaphoreGive(g_shared_memory.mutex);
    return true;
}

/**
 * Retrieve the latest sensor snapshot.
 * Called by CPU1 for telemetry reporting and state estimation.
 */
bool shared_memory_read_sensors(robot_sensor_data_t *data, TickType_t timeout)
{
    if (!g_initialized || data == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, timeout) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for sensor read (timeout)");
        return false;
    }

    *data = g_shared_memory.sensors;
    
    xSemaphoreGive(g_shared_memory.mutex);
    return true;
}

// =============================================================================
// Public API: Command Data (CPU1 -> CPU0)
// =============================================================================

/**
 * Update the last received command.
 * Called by CPU1 after receiving MQTT messages or API requests.
 */
bool shared_memory_write_command(const robot_command_t *cmd, TickType_t timeout)
{
    if (!g_initialized || cmd == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, timeout) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for command write (timeout)");
        return false;
    }

    g_shared_memory.last_command = *cmd;
    g_shared_memory.cpu1_alive = true;
    
    xSemaphoreGive(g_shared_memory.mutex);
    return true;
}

/**
 * Retrieve the pending control command.
 * Called by CPU0 to execute instructions from the Comms core.
 */
bool shared_memory_read_command(robot_command_t *cmd, TickType_t timeout)
{
    if (!g_initialized || cmd == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, timeout) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for command read (timeout)");
        return false;
    }

    *cmd = g_shared_memory.last_command;
    
    xSemaphoreGive(g_shared_memory.mutex);
    return true;
}

// =============================================================================
// Public API: Monitoring & Connectivity
// =============================================================================

void shared_memory_heartbeat_cpu0(void)
{
    if (!g_initialized) {
        return;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_shared_memory.heartbeat_cpu0++;
        xSemaphoreGive(g_shared_memory.mutex);
    }
}

void shared_memory_heartbeat_cpu1(void)
{
    if (!g_initialized) {
        return;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_shared_memory.heartbeat_cpu1++;
        xSemaphoreGive(g_shared_memory.mutex);
    }
}

void shared_memory_set_mqtt_connected(bool connected)
{
    if (!g_initialized) {
        return;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_shared_memory.mqtt_connected = connected;
        xSemaphoreGive(g_shared_memory.mutex);
    }
}

bool shared_memory_get_mqtt_connected(void)
{
    if (!g_initialized) {
        return false;
    }

    if (xSemaphoreTake(g_shared_memory.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool status = g_shared_memory.mqtt_connected;
        xSemaphoreGive(g_shared_memory.mutex);
        return status;
    }
    
    return false;
}

/**
 * Direct access to the shared structure (DANGER: use with caution).
 * Prefer using the read/write methods to ensure mutex protection.
 */
shared_memory_t* shared_memory_get(void)
{
    return g_initialized ? &g_shared_memory : NULL;
}
