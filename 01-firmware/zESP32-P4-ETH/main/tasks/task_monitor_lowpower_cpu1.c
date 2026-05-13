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
#include "performance_monitor.h"
#include "mqtt_custom_client.h"
#include <time.h>

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
    
    // Initialize required sensors and monitoring components
    test_sensor_init();
    perf_mon_init();

    if (ina_init() != ESP_OK) {
        ESP_LOGE(TAG, "INA hardware init failed");
    }

    // Load configuration from Kconfig
    uint32_t telemetry_rate_ms = 1000;
#ifdef CONFIG_INA226_TELEMETRY_RATE_MS
    telemetry_rate_ms = CONFIG_INA226_TELEMETRY_RATE_MS;
#endif

    const char* mqtt_topic_power = "robot/telemetry/power";
#ifdef CONFIG_INA226_MQTT_TOPIC
    mqtt_topic_power = CONFIG_INA226_MQTT_TOPIC;
#endif

    // Create telemetry instance for power metrics
    telemetry_handle_t telemetry_power = telemetry_create(mqtt_topic_power, "power_system", telemetry_rate_ms);
    telemetry_set_tags(telemetry_power, "sensor=battery");

    uint32_t polling_rate_hz = 5;
#ifdef CONFIG_INA226_POLLING_RATE_HZ
    polling_rate_hz = CONFIG_INA226_POLLING_RATE_HZ;
#endif
    if (polling_rate_hz == 0) polling_rate_hz = 1;

    // Grace period for network stabilization
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Counter for slower performance snapshots (e.g. every 5s)
    uint32_t perf_counter = 0;
    char uptime_str[64];

    while (1) {
        ina_data_t power_data = {0};
        
        // Capture analytics and perform safety checks
        ina_read(&power_data, true);

        // =====================================================================
        // Global State Update
        // =====================================================================
        shared_memory_t* shared_mem = shared_memory_get();
        if (shared_mem && xSemaphoreTake(shared_mem->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            shared_mem->sensors.battery_voltage = power_data.voltage_mv;
            shared_mem->sensors.robot_current   = power_data.current_ma;
            xSemaphoreGive(shared_mem->mutex);
        }

        // =====================================================================
        // Telemetry Reporting (Power)
        // =====================================================================
        if (telemetry_power) {
            telemetry_add_float(telemetry_power, "voltage_mv", power_data.voltage_mv);
            telemetry_add_float(telemetry_power, "current_ma", power_data.current_ma);
            telemetry_add_float(telemetry_power, "power_mw",   power_data.power_mw);
            telemetry_commit_point(telemetry_power);
        }

        // =====================================================================
        // Performance & Uptime Status (Periodic)
        // =====================================================================
        if (++perf_counter >= (5 * polling_rate_hz)) {
            perf_counter = 0;
            
            // 1. Print Uptime to Console (matching legacy behavior)
            test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
            ESP_LOGI(TAG, "System uptime: %s", uptime_str);

            // 2. Refresh performance snapshots
            if (perf_mon_update() == ESP_OK) {
                // 3. Print table to Console (matching legacy behavior)
                perf_mon_print_report();

                // 4. Publish Performance ILP to MQTT
                if (mqtt_custom_client_is_connected()) {
                    char ilp_buffer[1024];
                    struct timespec ts;
                    clock_gettime(CLOCK_REALTIME, &ts);
                    int64_t timestamp_ns = (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;

                    if (perf_mon_get_report_ilp(ilp_buffer, sizeof(ilp_buffer), timestamp_ns) == ESP_OK) {
                        mqtt_custom_client_publish("robot/telemetry/performance", ilp_buffer, 0, 0, 0);
                    }
                }
            }
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
