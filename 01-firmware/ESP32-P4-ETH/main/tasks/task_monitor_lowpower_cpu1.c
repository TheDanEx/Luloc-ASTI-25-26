/*
 * Task Monitor Low Power CPU1
 * Low-priority task for secondary monitoring only.
 * SPDX-License-Identifier: MIT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h" // For microsecond timestamps

#include "test_sensor.h"
#include "task_monitor_lowpower_cpu1.h"
#include "ina219_sensor.h"
#include "shared_memory.h"
#include "telemetry_manager.h"

static const char *TAG = "mon_cpu1";

static void task_monitor_lowpower_cpu1(void *arg)
{
    (void)arg;

    // 1. Initialize generic test sensor
    if (test_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize test sensor");
        // Proceed anyway, might just be a mock
    }

    // 2. Initialize INA219 Power Monitor
    if (ina219_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize INA219 sensor");
    }

    // 3. Create Telemetry Batched Reporter
    // This will aggregate data and send it every CONFIG_INA219_TELEMETRY_RATE_MS
    uint32_t telemetry_rate = 1000;
#ifdef CONFIG_INA219_TELEMETRY_RATE_MS
    telemetry_rate = CONFIG_INA219_TELEMETRY_RATE_MS;
#endif

    const char* topic = "robot/telemetry/power";
#ifdef CONFIG_INA219_MQTT_TOPIC
    topic = CONFIG_INA219_MQTT_TOPIC;
#endif

    telemetry_handle_t tel_handle = telemetry_create(topic, "power_system", telemetry_rate);

    // 4. Calculate sampling delay
    uint32_t polling_rate_hz = 5;
#ifdef CONFIG_INA219_POLLING_RATE_HZ
    polling_rate_hz = CONFIG_INA219_POLLING_RATE_HZ;
#endif
    if (polling_rate_hz == 0) polling_rate_hz = 1;
    TickType_t delay_ticks = pdMS_TO_TICKS(1000 / polling_rate_hz);

    char uptime_str[64];

    // Give system time to settle before looping
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        // --- Read INA219 ---
        float bus_voltage_mv = 0.0f;
        float current_ma = 0.0f;
        float power_mw = 0.0f;

        ina219_sensor_read_bus_voltage_mv(&bus_voltage_mv);
        ina219_sensor_read_current_ma(&current_ma);
        ina219_sensor_read_power_mw(&power_mw);

        // --- Save to Shared Memory (For other cores) ---
        shared_memory_t* shm = shared_memory_get();
        if (shm != NULL) {
            if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Update specific power fields
                shm->sensors.battery_voltage = bus_voltage_mv;
                shm->sensors.motor_current = current_ma;
                // Leave other sensor fields (encoders, speeds) untouched
                xSemaphoreGive(shm->mutex);
            }
        }

        // --- Add to Telemetry Batch (For MQTT) ---
        if (tel_handle) {
            telemetry_add_float(tel_handle, "voltage_mv", bus_voltage_mv);
            telemetry_add_float(tel_handle, "current_ma", current_ma);
            telemetry_add_float(tel_handle, "power_mw", power_mw);
            // Append timestamp dynamically per rule 5 (Influx Line Protocol)
            // Even though batched, we might want to know the last read time
            telemetry_add_int(tel_handle, "timestamp_us", (int32_t)esp_timer_get_time());
        }

        // --- Original Uptime Logging ---
        // (Reducing verbosity for clean console)
        /*
        test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
        ESP_LOGD(TAG, "Uptime: %s | V: %.0f mV, I: %.0f mA", uptime_str, bus_voltage_mv, current_ma);
        */

        vTaskDelay(delay_ticks);
    }
}

void task_monitor_lowpower_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_monitor_lowpower_cpu1, "monitor_cpu1", 4096, NULL, 1, NULL, 1);
}
