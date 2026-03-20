/*
 * Test Sensor Component - Simulates a sensor that tracks ESP32 uptime
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "test_sensor.h"

static const char *TAG = "test_sensor";

// Boot time reference (microseconds)
static int64_t boot_time_us = 0;

// =============================================================================
// Public API: Lifecycle
// =============================================================================

/**
 * Initialize the uptime-based virtual sensor.
 * Captures the boot-time reference using the ESP high-resolution timer.
 */
esp_err_t test_sensor_init(void)
{
    if (boot_time_us != 0) {
        ESP_LOGW(TAG, "Test sensor already initialized");
        return ESP_OK;
    }

    boot_time_us = esp_timer_get_time();
    return ESP_OK;
}

// =============================================================================
// Public API: Data Acquisition
// =============================================================================

/**
 * Perform a full read of the uptime metrics.
 * Populates millisecond, second, and minute variants for telemetry diversity.
 */
esp_err_t test_sensor_read(test_sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Test sensor not initialized");
        return ESP_FAIL;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;

    data->uptime_ms = elapsed_us / 1000;
    data->uptime_sec = elapsed_us / 1000000;
    data->uptime_min = (elapsed_us / 1000000) / 60;

    ESP_LOGD(TAG, "Uptime: %lu ms, %lu s, %lu min", 
             data->uptime_ms, data->uptime_sec, data->uptime_min);

    return ESP_OK;
}

uint32_t test_sensor_get_uptime_ms(void)
{
    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Test sensor not initialized");
        return 0;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;
    return (uint32_t)(elapsed_us / 1000);
}

uint32_t test_sensor_get_uptime_sec(void)
{
    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Test sensor not initialized");
        return 0;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;
    return (uint32_t)(elapsed_us / 1000000);
}

// =============================================================================
// Public API: Formatting
// =============================================================================

/**
 * Generate a human-readable HH:MM:SS string of the current uptime.
 * Useful for debugging logs and MQTT status messages.
 */
const char *test_sensor_get_uptime_str(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 16) return "ERROR";
    if (boot_time_us == 0) {
        snprintf(buffer, buffer_size, "NOT_INITIALIZED");
        return buffer;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;
    
    uint32_t total_sec = elapsed_us / 1000000;
    uint32_t hours = total_sec / 3600;
    uint32_t minutes = (total_sec % 3600) / 60;
    uint32_t seconds = total_sec % 60;
    uint32_t milliseconds = (elapsed_us % 1000000) / 1000;

    snprintf(buffer, buffer_size, "%luh %lum %lus %lums", 
             hours, minutes, seconds, milliseconds);

    return buffer;
}
