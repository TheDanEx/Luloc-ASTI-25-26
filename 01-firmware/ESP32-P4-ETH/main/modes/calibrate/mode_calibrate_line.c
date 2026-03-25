#include "mode_interface.h"
#include "line_sensor.h"
#include "task_telemetry.h"
#include "esp_log.h"
#include "state_machine.h"
#include "shared_memory.h"
#include "motor_velocity_ctrl.h"
#include <stdbool.h>
#include <string.h>

static const char *TAG = "MODE_CALIB_LINE";

static bool s_min_seen[8] = {false};
static bool s_max_seen[8] = {false};
static float s_timeout_timer = 0.0f;
static bool s_done = false;

static void calibrate_enter(void)
{
    ESP_LOGI(TAG, "Entering Line Calibration Mode");
    for (int i = 0; i < 8; i++) {
        s_min_seen[i] = false;
        s_max_seen[i] = false;
    }
    s_timeout_timer = 0.0f;
    s_done = false;
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

    // 2.1 Motor Control (Spin)
    float target_l = 0.0f;
    float target_r = 0.0f;

    if (!s_done) {
        if (!all_calibrated && s_timeout_timer < 10.0f) {
            target_l = 0.25f;  // Turn left
            target_r = -0.25f; // Turn right
            s_timeout_timer += dt_s;
        } else {
            s_done = true;
            if (all_calibrated) {
                ESP_LOGI(TAG, "Calibration successfully finished in %.2fs!", s_timeout_timer);
            } else {
                ESP_LOGW(TAG, "Calibration timed out after 10s!");
            }
        }
    }

    // Use shared memory for current values
    shared_memory_t* shm = shared_memory_get();
    float bat_mv = 12000.0f;
    float cur_l = 0.0f, cur_r = 0.0f;
    
    if (xSemaphoreTake(shm->mutex, 0) == pdTRUE) {
        bat_mv = shm->sensors.battery_voltage;
        cur_l = shm->sensors.motor_speed_left;
        cur_r = shm->sensors.motor_speed_right;
        if (bat_mv < 5000.0f) bat_mv = 16800.0f;
        xSemaphoreGive(shm->mutex);
    }

    float pwm_l, pwm_r;
    motor_velocity_input_t in_l = { .target_speed = target_l, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t in_r = { .target_speed = target_r, .current_speed = cur_r, .battery_mv = bat_mv };
    
    motor_velocity_ctrl_update(ctrl_left, &in_l, dt_s, &pwm_l, NULL);
    motor_velocity_ctrl_update(ctrl_right, &in_r, dt_s, &pwm_r, NULL);
    motor_mcpwm_set(motors, (int16_t)(pwm_l * 10), (int16_t)(pwm_r * 10));

    // 3. Send Telemetry
    line_follower_telemetry_t tele = {
        .mode = MODE_CALIBRATE_LINE,
        .position = 0.0f
    };
    memcpy(tele.raw, raw, sizeof(raw));
    line_sensor_read_norm(tele.norm, raw, 16);
    task_telemetry_send(&tele);

    // 4. State Transition when finished
    if (s_done) {
        state_machine_request_mode(MODE_NONE, false);
    }
}

static void calibrate_exit(motor_driver_mcpwm_t* motors)
{
    ESP_LOGI(TAG, "Exiting Line Calibration Mode");
    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_calibrate_line = {
    .enter = calibrate_enter,
    .execute = calibrate_execute,
    .exit = calibrate_exit
};
