#include "motor_velocity_ctrl.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const char *TAG = "VEL_CTRL";

struct motor_velocity_ctrl_context_t {
    motor_velocity_config_t config;
    float integral;
    float prev_filtered_speed;
    float current_filtered_speed;
    float ramped_target_speed;
    float filtered_derivative;
    float last_out_pwm;
};

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

esp_err_t motor_velocity_ctrl_create(const motor_velocity_config_t *config, motor_velocity_ctrl_handle_t *out_handle) {
    if (config == NULL || out_handle == NULL) return ESP_ERR_INVALID_ARG;
    struct motor_velocity_ctrl_context_t *ctx = calloc(1, sizeof(struct motor_velocity_ctrl_context_t));
    if (ctx == NULL) return ESP_ERR_NO_MEM;
    ctx->config = *config;
    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t motor_velocity_ctrl_destroy(motor_velocity_ctrl_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    free(handle);
    return ESP_OK;
}

esp_err_t motor_velocity_ctrl_update(motor_velocity_ctrl_handle_t handle, 
                                     const motor_velocity_input_t *input, 
                                     float delta_time_s,
                                     float *out_pwm_duty,
                                     motor_velocity_diag_t *out_diag) {
    if (handle == NULL || input == NULL || out_pwm_duty == NULL) return ESP_ERR_INVALID_ARG;
    if (delta_time_s <= 0.0f) delta_time_s = 0.01f;

    struct motor_velocity_ctrl_context_t *ctx = handle;
    float battery_v = input->battery_mv / 1000.0f;
    if (battery_v < 1.0f) battery_v = ctx->config.max_battery_mv / 1000.0f;

    // 1. Signal Filtering
    ctx->prev_filtered_speed = ctx->current_filtered_speed;
    ctx->current_filtered_speed = (ctx->config.ema_alpha * input->current_speed) + 
                                  ((1.0f - ctx->config.ema_alpha) * ctx->current_filtered_speed);

    // 2. Motion Generation (Ramps)
    float max_step = ctx->config.accel_limit_ms2 * delta_time_s;
    float speed_diff = input->target_speed - ctx->ramped_target_speed;
    ctx->ramped_target_speed += clamp(speed_diff, -max_step, max_step);

    // 3. PID Correction (D-on-PV)
    float error = ctx->ramped_target_speed - ctx->current_filtered_speed;
    if (fabsf(ctx->ramped_target_speed) < 0.001f) {
        ctx->integral = 0.0f;
    } else {
        // Smart Anti-Windup: Pause integration if we are far from the target (e.g. accelerating hard)
        // This prevents the "overshoot hump" from accumulating too much phantom voltage.
        if (fabsf(error) < 0.3f) {
            ctx->integral += error * delta_time_s;
            ctx->integral = clamp(ctx->integral, -5.0f, 5.0f);
        }
    }
    
    // Low-Pass Filter on the Derivative to kill the 500Hz quantization noise
    float raw_derivative = (ctx->current_filtered_speed - ctx->prev_filtered_speed) / delta_time_s;
    ctx->filtered_derivative = (0.2f * raw_derivative) + (0.8f * ctx->filtered_derivative);
    
    float p_v = ctx->config.kp * error;
    float i_v = ctx->config.ki * ctx->integral;
    float d_v = -(ctx->config.kd * ctx->filtered_derivative); // Smoothed D-on-PV
    float pid_voltage = p_v + i_v + d_v;

    // 4. Feed-Forward & Deadband
    float ff_voltage = 0.0f;
    if (fabsf(ctx->ramped_target_speed) > 0.001f) {
        float base_v = (ctx->ramped_target_speed / ctx->config.max_motor_speed) * (ctx->config.max_battery_mv / 1000.0f);
        float deadband_offset = (ctx->ramped_target_speed > 0) ? ctx->config.deadband_v : -ctx->config.deadband_v;
        ff_voltage = base_v + deadband_offset;
    }
    float target_voltage_v = ff_voltage + pid_voltage;

    // 5. Saturation & Raw PWM
    if (fabsf(target_voltage_v) > battery_v) {
        target_voltage_v = (target_voltage_v > 0) ? battery_v : -battery_v;
    }
    float raw_pwm_duty = (target_voltage_v / battery_v) * 100.0f;
    raw_pwm_duty = clamp(raw_pwm_duty, -100.0f, 100.0f);

    // 6. Slew Rate Limiter (Max PWM Delta per Cycle)
    // Constraint: 500.0% PWM change per second. (At 2ms cycle = max 1.0% jump per frame).
    float max_pwm_step = 500.0f * delta_time_s;
    float pwm_diff = raw_pwm_duty - ctx->last_out_pwm;
    
    // Si la diferencia excede el límite permitido por frame, la frenamos
    float limited_pwm_duty = ctx->last_out_pwm + clamp(pwm_diff, -max_pwm_step, max_pwm_step);
    
    *out_pwm_duty = limited_pwm_duty;
    ctx->last_out_pwm = limited_pwm_duty;

    // 6. Fill Diagnostics
    if (out_diag) {
        out_diag->target_ramped = ctx->ramped_target_speed;
        out_diag->error = error;
        out_diag->feed_forward_v = ff_voltage;
        out_diag->p_v = p_v;
        out_diag->i_v = i_v;
        out_diag->d_v = d_v;
        out_diag->final_v = target_voltage_v;
        out_diag->pwm_duty = limited_pwm_duty;
    }

    return ESP_OK;
}

esp_err_t motor_velocity_ctrl_set_pid(motor_velocity_ctrl_handle_t handle, float kp, float ki, float kd) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    struct motor_velocity_ctrl_context_t *ctx = handle;
    ctx->config.kp = kp;
    ctx->config.ki = ki;
    ctx->config.kd = kd;
    ctx->integral = 0.0f;
    ctx->filtered_derivative = 0.0f;
    return ESP_OK;
}
