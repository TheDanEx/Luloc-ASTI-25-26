#include "mode_interface.h"
#include "line_sensor.h"
#include "task_telemetry.h"
#include "esp_log.h"
#include "state_machine.h"
#include "shared_memory.h"
#include "motor_velocity_ctrl.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

static const char *TAG = "MODE_CALIB_LINE";

static bool s_min_seen[8] = {false};
static bool s_max_seen[8] = {false};
static float s_timeout_timer = 0.0f;
static float s_last_print_time = -0.6f;
static bool s_done = false;

static void calibrate_enter(void)
{
    ESP_LOGI(TAG, "Entering Line Calibration Mode");
    for (int i = 0; i < 8; i++) {
        s_min_seen[i] = false;
        s_max_seen[i] = false;
        // Reset calibration to extreme values to start fresh
        line_sensor_set_calibration(i, 4095, 0);
    }
    s_timeout_timer = 0.0f;
    s_last_print_time = -0.6f;
    s_done = false;
}

static void calibrate_execute(motor_driver_mcpwm_t* motors, 
                             motor_velocity_ctrl_handle_t ctrl_left, 
                             motor_velocity_ctrl_handle_t ctrl_right, 
                             float dt_s)
{
    // 1. Read Raw Values (Background sampling is on CPU1, but raw can be read directly)
    uint32_t raw[8];
    line_sensor_read_raw(raw, 1); 

    // 2. Update Max/Min registers and check contrast
    bool all_contrasted = true;
    for (int i = 0; i < 8; i++) {
        uint32_t c_min, c_max;
        line_sensor_get_calibration(i, &c_min, &c_max);

        if (raw[i] < c_min) c_min = raw[i];
        if (raw[i] > c_max) c_max = raw[i];

        line_sensor_set_calibration(i, c_min, c_max);

        // Verification logic
        if (i >= 2 && i <= 5) { // Analog center sensors
            uint32_t range = (c_max > c_min) ? (c_max - c_min) : 0;
            if (range >= CONFIG_LINE_SENSOR_CALIB_MIN_RANGE) {
                s_min_seen[i] = true;
                s_max_seen[i] = true;
            }
        } else { // Digital edge sensors (just need to see both states)
            if (raw[i] == 0) s_min_seen[i] = true;
            if (raw[i] == 1) s_max_seen[i] = true;
        }

        if (!s_min_seen[i] || !s_max_seen[i]) {
            all_contrasted = false;
        }
    }

    s_timeout_timer += dt_s;

    // 2.1 Serial monitor periodic log
    if (s_timeout_timer - s_last_print_time >= 0.5f) {
        s_last_print_time = s_timeout_timer;
        printf("\r\n--- CALIBRATING [T:%.1f/10.0s] ---\r\n", s_timeout_timer);
        printf("SNR | RAW  | MIN  | MAX  | RANGE | OK?\r\n");
        for (int i = 0; i < 8; i++) {
            uint32_t mi, ma;
            line_sensor_get_calibration(i, &mi, &ma);
            uint32_t r = (ma > mi) ? (ma - mi) : 0;
            printf("S%d  | %4lu | %4lu | %4lu | %5lu | %s\r\n", 
                   i, raw[i], mi, ma, (unsigned long)r, 
                   (s_min_seen[i] && s_max_seen[i]) ? "YES" : "NO");
        }
    }

    // 3. Control (Spin)
    float target_l = 0.0f;
    float target_r = 0.0f;

    // Termination condition: At least 10s AND all sensors contrasted (or 30s timeout)
    if (!s_done) {
        if (s_timeout_timer < 10.0f || (!all_contrasted && s_timeout_timer < 30.0f)) {
            target_l = 0.20f;  // Slow spin for better sampling
            target_r = -0.20f;
        } else {
            s_done = true;
            if (all_contrasted) {
                ESP_LOGI(TAG, "Calibration SUCCESS after %.1f s", s_timeout_timer);
            } else {
                ESP_LOGE(TAG, "Calibration FAILED (Contrast Range < %d) after 30s", CONFIG_LINE_SENSOR_CALIB_MIN_RANGE);
            }
        }
    }

    // Get feedback from encoders and battery
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

    // 4. Send Telemetry including MIN/MAX
    line_follower_telemetry_t tele = {
        .mode = MODE_CALIBRATE_LINE,
        .position = 0.0f
    };
    memcpy(tele.raw, raw, sizeof(raw));
    for(int i=0; i<8; i++) {
        line_sensor_get_calibration(i, &tele.min[i], &tele.max[i]);
    }
    task_telemetry_send(&tele);

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
