#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "uros_network_interfaces.h"
#include "shared_memory.h"
#include "uros_manager.h"

static const char *TAG = "MAIN";

// =============================================================================
// Core 1: Real-Time Control (Dummy Example)
// =============================================================================

void task_control_core1(void *pvParameters) {
    ESP_LOGI(TAG, "Starting Control Task on Core 1");
    
    robot_telemetry_fast_t fast_data = {0};
    robot_telemetry_slow_t slow_data = {0};
    
    robot_mode_cmd_t mode_cmd;
    robot_twist_cmd_t twist_cmd;

    // Initial placeholder calibration
    for(int i=0; i<8; i++) {
        slow_data.calib_min[i] = 100.0f;
        slow_data.calib_max[i] = 900.0f;
    }

    uint64_t last_time = esp_timer_get_time();

    while (1) {
        // 1. Read Commands
        if (shared_memory_get_mode_cmd(&mode_cmd) == ESP_OK) {
            slow_data.current_mode = mode_cmd.new_mode;
            ESP_LOGI("CTRL", "Mode changed to: %lu", (unsigned long)slow_data.current_mode);
        }

        if (shared_memory_get_twist_cmd(&twist_cmd) == ESP_OK) {
            fast_data.pid_setpoint = twist_cmd.linear_x;
        }

        // 2. Simulate Sensor Data (Placeholder)
        for(int i=0; i<8; i++) {
            fast_data.line_raw[i] = 200.0f + (float)(rand() % 50);
            fast_data.line_norm[i] = (fast_data.line_raw[i] - slow_data.calib_min[i]) / (slow_data.calib_max[i] - slow_data.calib_min[i]);
        }

        // 3. Simulate PID Logic (Placeholder)
        fast_data.pid_error = fast_data.pid_setpoint - fast_data.pid_output;
        fast_data.pid_p = fast_data.pid_error * 0.5f;
        fast_data.pid_i += fast_data.pid_error * 0.01f;
        fast_data.pid_output = fast_data.pid_p + fast_data.pid_i;

        // 4. Simulate Odometry (Placeholder)
        slow_data.odom_lin = fast_data.pid_output;
        slow_data.odom_x += slow_data.odom_lin * 0.01f;
        slow_data.odom_theta += twist_cmd.angular_z * 0.01f;

        // 5. Push Data
        shared_memory_push_telemetry_fast(&fast_data);
        
        static int slow_counter = 0;
        if (slow_counter++ >= 100) { // 1Hz
            slow_counter = 0;
            slow_data.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
            slow_data.battery_v = 12.6f;
            shared_memory_push_telemetry_slow(&slow_data);
        }

        vTaskDelayUntil((TickType_t*)&last_time, pdMS_TO_TICKS(10)); // 100Hz
    }
}

// =============================================================================
// App Main
// =============================================================================

void app_main(void) {
    ESP_ERROR_CHECK(shared_memory_init());
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(uros_manager_start());

    xTaskCreatePinnedToCore(task_control_core1, "control_task", 4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG, "System started successfully");
}
