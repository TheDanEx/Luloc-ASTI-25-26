#include "mode_interface.h"
#include "line_sensor.h"
#include "task_telemetry.h"
#include "esp_log.h"
#include "motor_velocity_ctrl.h"
#include "shared_memory.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "MODE_FOLLOW_LINE";

static float s_kp = 0.05f;
static float s_ki = 0.0f;
static float s_kd = 0.01f;
static float s_base_speed = 0.3f;
static float s_ff_weight = 1.0f;
static float s_ff_value = 1.0f;
static float s_min_speed = 0.1f;
static float s_max_speed = 1.5f;

static float s_integral = 0.0f;
static float s_last_error = 0.0f;
static float s_last_pos = 0.0f;

static void follow_enter(void)
{
    ESP_LOGI(TAG, "Entering Line Follower Mode");
    s_integral = 0.0f;
    s_last_error = 0.0f;
    s_last_pos = 0.0f;

    // Load parameters from Kconfig
    s_kp = atof(CONFIG_LINE_FOLLOWER_DEFAULT_KP);
    s_ki = atof(CONFIG_LINE_FOLLOWER_DEFAULT_KI);
    s_kd = atof(CONFIG_LINE_FOLLOWER_DEFAULT_KD);
    s_base_speed = atof(CONFIG_LINE_FOLLOWER_BASE_SPEED);
    s_ff_weight = atof(CONFIG_LINE_FOLLOWER_FF_WEIGHT);
    s_ff_value = atof(CONFIG_LINE_FOLLOWER_FF_VALUE);
    s_min_speed = atof(CONFIG_LINE_FOLLOWER_MIN_SPEED);
    s_max_speed = atof(CONFIG_LINE_FOLLOWER_MAX_SPEED);

    // 0. Calibration Check (Warning only)
    uint32_t c_min, c_max;
    // Check D4 (index 3) as a representative sensor
    line_sensor_get_calibration(3, &c_min, &c_max);
    if (c_min == 100 && c_max == 4000) { // Using standard Kconfig defaults as markers
        ESP_LOGW(TAG, "Robot has NOT been calibrated! Sensors will behave erratically.");
    }
}

static void follow_execute(motor_driver_mcpwm_t* motors, 
                          motor_velocity_ctrl_handle_t ctrl_left, 
                          motor_velocity_ctrl_handle_t ctrl_right, 
                          float dt_s)
{
    uint32_t start_us = (uint32_t)esp_timer_get_time();

    // 1. Read Live Parameters from Shared Memory
    shared_memory_t* shm = shared_memory_get();
    if (shm->line_pid.updated_flag) {
        s_kp = shm->line_pid.kp;
        s_ki = shm->line_pid.ki;
        s_kd = shm->line_pid.kd;
        shm->line_pid.updated_flag = false;
        ESP_LOGI(TAG, "Line PID Updated: %.3f, %.3f, %.3f", s_kp, s_ki, s_kd);
    }
    if (shm->line_params_updated) {
        s_base_speed = shm->line_base_speed;
        shm->line_params_updated = false;
        ESP_LOGI(TAG, "Line Base Speed Updated: %.3f", s_base_speed);
    }

    // 1.1 Read Asynchronous Sensor Data (CPU1 Sampled)
    float cur_l = 0.0f, cur_r = 0.0f, bat_mv = 12000.0f;
    float norm[8];
    
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        s_ff_value = shm->vision_curvature_multiplier;
        cur_l = shm->sensors.motor_speed_left;
        cur_r = shm->sensors.motor_speed_right;
        bat_mv = shm->sensors.battery_voltage;
        memcpy(norm, shm->sensors.line_norm, sizeof(norm));
        
        if (bat_mv < 5000.0f) bat_mv = 16800.0f; // Safety fallback
        xSemaphoreGive(shm->mutex);
    }

    // 2. Calculate Position (Using pre-normalized values from memory)
    float pos = line_sensor_read_line_position(norm, 0); // num_samples=0 forces use of norm
    
    // Edge case: No line detected
    bool line_lost = true;
    for(int i=0; i<8; i++) if(norm[i] > 0.5f) line_lost = false;

    if (line_lost) {
        // Assume the line is at the "next" virtual sensor (10mm beyond the last seen)
        pos = s_last_pos + (s_last_pos > 0 ? 10.0f : -10.0f);
    } else {
        // Only update memory when we HAVE a valid line
        s_last_pos = pos;
    }

    // 2. Control PID
    float error = 0.0f - pos;
    s_integral += error * dt_s;
    float derivative = (error - s_last_error) / dt_s;
    s_last_error = error;

    float p_term = s_kp * error;
    float i_term = s_ki * s_integral;
    float d_term = s_kd * derivative;
    float pid_out = p_term + i_term + d_term;
    
    if (pid_out > 1.0f) pid_out = 1.0f;
    if (pid_out < -1.0f) pid_out = -1.0f;

    // 3. Kinematics (Feedforward + Differential)
    float effective_multiplier = 1.0f + (s_ff_value - 1.0f) * s_ff_weight;
    float speed_base_ff = s_base_speed * effective_multiplier;
    
    // Differential Steering (Additive correction to allow rotation at base_speed = 0)
    float target_l = speed_base_ff + pid_out;
    float target_r = speed_base_ff - pid_out;

    // 3.1 Advanced Minimal Speed Logic (Improved per user request)
    // Only apply if the target speed is non-zero (absolute)
    if (fabsf(target_l) > 0.001f) {
        if (target_l > 0 && target_l < s_min_speed) target_l = s_min_speed;
        else if (target_l < 0 && target_l > -s_min_speed) target_l = -s_min_speed;
    }
    if (fabsf(target_r) > 0.001f) {
        if (target_r > 0 && target_r < s_min_speed) target_r = s_min_speed;
        else if (target_r < 0 && target_r > -s_min_speed) target_r = -s_min_speed;
    }

    // Clamp wheel targets to hardware limits
    if (target_l > s_max_speed) target_l = s_max_speed;
    if (target_l < -s_max_speed) target_l = -s_max_speed;
    if (target_r > s_max_speed) target_r = s_max_speed;
    if (target_r < -s_max_speed) target_r = -s_max_speed;

    // 4. Actuation
    float pwm_l, pwm_r;
    motor_velocity_input_t in_l = { .target_speed = target_l, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t in_r = { .target_speed = target_r, .current_speed = cur_r, .battery_mv = bat_mv };
    
    motor_velocity_ctrl_update(ctrl_left, &in_l, dt_s, &pwm_l, NULL);
    motor_velocity_ctrl_update(ctrl_right, &in_r, dt_s, &pwm_r, NULL);
    motor_mcpwm_set(motors, (int16_t)(pwm_l * 10), (int16_t)(pwm_r * 10));

    uint32_t end_us = (uint32_t)esp_timer_get_time();

    // 5. Telemetry (Decimated at 50Hz)
    static uint8_t telemetry_div = 0;
    if (telemetry_div++ >= 10) {
        telemetry_div = 0;
        line_follower_telemetry_t tele = {
            .mode = MODE_FOLLOW_LINE,
            .position = pos,
            .error = error,
            .p_term = p_term, .i_term = i_term, .d_term = d_term,
            .kp = s_kp, .ki = s_ki, .kd = s_kd,
            .pid_out = pid_out, .ff_val = s_ff_value,
            .target_speed_l = target_l,
            .target_speed_r = target_r,
            .cycle_time_us = (float)(end_us - start_us)
        };
        memcpy(tele.norm, norm, sizeof(norm));
        task_telemetry_send(&tele);
    }
}

static void follow_exit(motor_driver_mcpwm_t* motors)
{
    ESP_LOGI(TAG, "Exiting Line Follower Mode");
    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_follow_line = {
    .enter = follow_enter,
    .execute = follow_execute,
    .exit = follow_exit
};
