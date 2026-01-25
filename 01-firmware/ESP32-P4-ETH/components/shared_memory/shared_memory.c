#include "shared_memory.h"
#include "esp_log.h"

static const char *TAG = "shrd_mem";

// Global shared memory instance
static shared_memory_t g_shared_memory = {0};
static bool g_initialized = false;

void shared_memory_init(void)
{
    if (g_initialized) {
        return;
    }

    // Initialize mutex for thread-safe access
    g_shared_memory.mutex = xSemaphoreCreateMutex();
    if (g_shared_memory.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Initialize shared data to zero
    g_shared_memory.sensors = (robot_sensor_data_t){0};
    g_shared_memory.last_command = (robot_command_t){0};
    g_shared_memory.heartbeat_cpu0 = 0;
    g_shared_memory.heartbeat_cpu1 = 0;
    g_shared_memory.cpu0_alive = false;
    g_shared_memory.cpu1_alive = false;

    g_initialized = true;
    ESP_LOGI(TAG, "Shared memory initialized");
}

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

shared_memory_t* shared_memory_get(void)
{
    return g_initialized ? &g_shared_memory : NULL;
}
