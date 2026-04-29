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
    
    robot_line_sensors_t line_sensors = {0};
    robot_calibration_t calibration = {0};
    robot_pid_data_t pid_data = {0};
    robot_odometry_t odometry = {0};
    robot_state_t state = { .current_mode = 0, .battery_voltage = 12.6f, .uptime_s = 0 };
    
    robot_mode_cmd_t mode_cmd;
    robot_twist_cmd_t twist_cmd;

    // Set some initial placeholder calibration values
    for(int i=0; i<8; i++) {
        calibration.min[i] = 100.0f;
        calibration.max[i] = 900.0f;
    }

    uint64_t last_time = esp_timer_get_time();

    while (1) {
        // 1. Read Commands
        if (shared_memory_get_mode_cmd(&mode_cmd) == ESP_OK) {
            state.current_mode = mode_cmd.new_mode;
            ESP_LOGI("CTRL", "Mode changed to: %d", state.current_mode);
        }

        if (shared_memory_get_twist_cmd(&twist_cmd) == ESP_OK) {
            pid_data.setpoint = twist_cmd.linear_x;
        }

        // 2. Simulate Sensor Data (Placeholder)
        for(int i=0; i<8; i++) {
            line_sensors.raw[i] = 200.0f + (float)(rand() % 50);
            line_sensors.normalized[i] = (line_sensors.raw[i] - calibration.min[i]) / (calibration.max[i] - calibration.min[i]);
        }

        // 3. Simulate PID Logic (Placeholder)
        pid_data.error = pid_data.setpoint - pid_data.output;
        pid_data.p_term = pid_data.error * 0.5f;
        pid_data.i_term += pid_data.error * 0.01f;
        pid_data.d_term = 0.0f;
        pid_data.output = pid_data.p_term + pid_data.i_term + pid_data.d_term;

        // 4. Simulate Odometry (Placeholder)
        odometry.vel_linear = pid_data.output;
        odometry.pos_x += odometry.vel_linear * 0.01f;
        odometry.theta += twist_cmd.angular_z * 0.01f;

        // 5. Push Data to Shared Memory
        shared_memory_push_line_sensors(&line_sensors);
        shared_memory_push_pid_data(&pid_data);
        shared_memory_push_odometry(&odometry);
        shared_memory_push_calibration(&calibration);
        
        static int state_counter = 0;
        if (state_counter++ >= 100) { // 1Hz
            state_counter = 0;
            state.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
            shared_memory_push_state(&state);
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

    // Priority 10 on Core 1 for Control Task
    xTaskCreatePinnedToCore(task_control_core1, "control_task", 4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG, "System started successfully");
}
