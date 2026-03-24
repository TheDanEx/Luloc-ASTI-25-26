#include "mode_interface.h"
#include "line_sensor.h"
#include "task_telemetry.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "MODE_CALIB_LINE";

static bool s_min_seen[8] = {false};
static bool s_max_seen[8] = {false};

static void calibrate_enter(void)
{
    ESP_LOGI(TAG, "Entering Line Calibration Mode");
    for (int i = 0; i < 8; i++) {
        s_min_seen[i] = false;
        s_max_seen[i] = false;
    }
}

static void calibrate_execute(motor_driver_mcpwm_t* motors, 
                             motor_velocity_ctrl_handle_t ctrl_left, 
                             motor_velocity_ctrl_handle_t ctrl_right, 
                             float dt_s)
{
    // 1. Read Raw Values
    uint32_t raw[8];
    line_sensor_read_raw(raw, 16);

    // 2. Update Max/Min registers
    bool all_calibrated = true;
    for (int i = 0; i < 8; i++) {
        uint32_t current_min, current_max;
        line_sensor_get_calibration(i, &current_min, &current_max);

        if (raw[i] < current_min) current_min = raw[i];
        if (raw[i] > current_max) current_max = raw[i];

        line_sensor_set_calibration(i, current_min, current_max);

        // Check conditions: Analog sensors >65% and <35% of absolute range
        // ADC range is 0..4095
        if (raw[i] < 1433) s_min_seen[i] = true; // < 35%
        if (raw[i] > 2662) s_max_seen[i] = true; // > 65%

        if (!s_min_seen[i] || !s_max_seen[i]) {
            all_calibrated = false;
        }
    }

    // 3. Send Telemetry
    line_follower_telemetry_t tele = {
        .mode = MODE_CALIBRATE_LINE,
        .position = 0.0f
    };
    memcpy(tele.raw, raw, sizeof(raw));
    line_sensor_read_norm(tele.norm, raw, 16);
    task_telemetry_send(&tele);

    // 4. Check if finished (optional: trigger state transition)
    if (all_calibrated) {
        ESP_LOGI(TAG, "Calibration finished successfully!");
        // TODO: Transition to IDLE or WAITING
    }
}

static void calibrate_exit(motor_driver_mcpwm_t* motors)
{
    ESP_LOGI(TAG, "Exiting Line Calibration Mode");
}

const mode_interface_t mode_calibrate_line = {
    .enter = calibrate_enter,
    .execute = calibrate_execute,
    .exit = calibrate_exit
};
