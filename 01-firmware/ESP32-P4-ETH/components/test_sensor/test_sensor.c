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

/**
 * @brief Initialize test sensor
 */
esp_err_t test_sensor_init(void)
{
    if (boot_time_us != 0) {
        ESP_LOGW(TAG, "Test sensor already initialized");
        return ESP_OK;
    }

    // Get current time as reference point (when ESP booted)
    boot_time_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Test sensor initialized - tracking uptime");
    return ESP_OK;
}

/**
 * @brief Get current sensor reading (uptime)
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

    // Calculate elapsed time since boot
    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;

    // Convert to different units
    data->uptime_ms = elapsed_us / 1000;
    data->uptime_sec = elapsed_us / 1000000;
    data->uptime_min = (elapsed_us / 1000000) / 60;

    ESP_LOGD(TAG, "Uptime: %lu ms, %lu s, %lu min", 
             data->uptime_ms, data->uptime_sec, data->uptime_min);

    return ESP_OK;
}

/**
 * @brief Get uptime in milliseconds
 */
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

/**
 * @brief Get uptime in seconds
 */
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

/**
 * @brief Get uptime as formatted string
 */
const char *test_sensor_get_uptime_str(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 16) {
        return "ERROR";
    }

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
