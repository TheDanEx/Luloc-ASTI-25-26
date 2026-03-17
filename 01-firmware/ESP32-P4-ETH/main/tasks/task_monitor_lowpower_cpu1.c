/*
 * Task Monitor Low Power CPU1
 * SPDX-License-Identifier: MIT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "test_sensor.h"
#include "task_monitor_lowpower_cpu1.h"
#include "ina226_sensor.h"
#include "shared_memory.h"
#include "telemetry_manager.h"

// =============================================================================
// Constants & Config
// =============================================================================
static const char *TAG = "mon_cpu1";

// =============================================================================
// Main Task Implementation
// =============================================================================

static void task_monitor_lowpower_cpu1(void *arg)
{
    (void)arg;
    test_sensor_init();

    if (ina_init() != ESP_OK) {
        ESP_LOGE(TAG, "INA hardware init failed");
    }

    uint32_t telemetry_rate_ms = 1000;
#ifdef CONFIG_INA226_TELEMETRY_RATE_MS
    telemetry_rate_ms = CONFIG_INA226_TELEMETRY_RATE_MS;
#endif

    const char* mqtt_topic_power = "robot/telemetry/power";
#ifdef CONFIG_INA226_MQTT_TOPIC
    mqtt_topic_power = CONFIG_INA226_MQTT_TOPIC;
#endif

    telemetry_handle_t telemetry_power = telemetry_create(mqtt_topic_power, "power_system", telemetry_rate_ms);

    uint32_t polling_rate_hz = 5;
#ifdef CONFIG_INA226_POLLING_RATE_HZ
    polling_rate_hz = CONFIG_INA226_POLLING_RATE_HZ;
#endif
    if (polling_rate_hz == 0) polling_rate_hz = 1;

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        ina_data_t power_data = {0};
        
        // Capture data and trigger audible alerts if thresholds are breached
        ina_read(&power_data, true);

        // Update Global State (Shared Memory)
        shared_memory_t* shared_mem = shared_memory_get();
        if (shared_mem && xSemaphoreTake(shared_mem->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            shared_mem->sensors.battery_voltage = power_data.voltage_mv;
            shared_mem->sensors.robot_current   = power_data.current_ma;
            xSemaphoreGive(shared_mem->mutex);
        }

        // Send to Telemetry Manager (MQTT Batching)
        if (telemetry_power) {
            telemetry_add_float(telemetry_power, "voltage_mv", power_data.voltage_mv);
            telemetry_add_float(telemetry_power, "current_ma", power_data.current_ma);
            telemetry_add_float(telemetry_power, "power_mw",   power_data.power_mw);
            telemetry_add_int(telemetry_power,   "timestamp_us", (int32_t)esp_timer_get_time());
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / polling_rate_hz));
    }
}

// =============================================================================
// Start Task Wrapper
// =============================================================================

void task_monitor_lowpower_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_monitor_lowpower_cpu1, "monitor_cpu1", 4096, NULL, 1, NULL, 1);
}
