#include "motor_velocity_ctrl.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "VEL_CTRL";

struct motor_velocity_ctrl_context_t {
    motor_velocity_config_t config;
    float integral;
    float previous_error;
};

esp_err_t motor_velocity_ctrl_create(const motor_velocity_config_t *config, motor_velocity_ctrl_handle_t *out_handle) {
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct motor_velocity_ctrl_context_t *ctx = calloc(1, sizeof(struct motor_velocity_ctrl_context_t));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for velocity controller");
        return ESP_ERR_NO_MEM;
    }

    ctx->config = *config;
    ctx->integral = 0.0f;
    ctx->previous_error = 0.0f;

    *out_handle = ctx;
    ESP_LOGI(TAG, "Velocity controller created.");
    return ESP_OK;
}

esp_err_t motor_velocity_ctrl_destroy(motor_velocity_ctrl_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    free(handle);
    return ESP_OK;
}

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

esp_err_t motor_velocity_ctrl_update(motor_velocity_ctrl_handle_t handle, 
                                     const motor_velocity_input_t *input, 
                                     float delta_time_s,
                                     float *out_pwm_duty) {
    if (handle == NULL || input == NULL || out_pwm_duty == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (delta_time_s <= 0.0f) {
        delta_time_s = 0.01f;
    }

    struct motor_velocity_ctrl_context_t *ctx = handle;

    // --- 1. Feed-Forward Calculation (Battery Compensation) ---
    // How much PWM is theoretically required to reach target_speed 
    // given the current battery state?

    // Prevent division by zero if battery is dead or unconnected
    float safe_battery_mv = (input->battery_mv > 5000.0f) ? input->battery_mv : ctx->config.max_battery_mv;

    // Base fraction of max speed
    float speed_fraction = input->target_speed / ctx->config.max_motor_speed; 
    
    // Theoretical PWM if battery was perfect (16.8V for 4S)
    float base_pwm = speed_fraction * 100.0f; 
    
    // Scale up the PWM if the battery is depleted
    // E.g. If battery is 14V instead of 16.8V, we need (16.8/14) = 1.2x more PWM effort.
    float voltage_compensator = ctx->config.max_battery_mv / safe_battery_mv;
    float feed_forward_pwm = base_pwm * voltage_compensator;


    // --- 2. PID Correction (Closed-Loop) ---
    float error = input->target_speed - input->current_speed;
    
    // Only accumulate integral if target is not zero and we are not saturated
    // Basic anti-windup:
    if (input->target_speed != 0.0f) {
        ctx->integral += error * delta_time_s;
    } else {
        ctx->integral = 0.0f; 
    }

    float derivative = (error - ctx->previous_error) / delta_time_s;

    float pid_correction_pwm = (ctx->config.kp * error) + 
                               (ctx->config.ki * ctx->integral) + 
                               (ctx->config.kd * derivative);

    ctx->previous_error = error;

    
    // --- 3. Final Output Combination ---
    // If target is perfectly 0, we can bypass math and output 0 to ensure crisp stops
    if (input->target_speed == 0.0f && pid_correction_pwm < 5.0f && pid_correction_pwm > -5.0f) {
        *out_pwm_duty = 0.0f;
        return ESP_OK;
    }

    float final_pwm = feed_forward_pwm + pid_correction_pwm;
    *out_pwm_duty = clamp(final_pwm, -100.0f, 100.0f);

    return ESP_OK;
}

esp_err_t motor_velocity_ctrl_set_pid(motor_velocity_ctrl_handle_t handle, float kp, float ki, float kd) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    
    struct motor_velocity_ctrl_context_t *ctx = handle;
    ctx->config.kp = kp;
    ctx->config.ki = ki;
    ctx->config.kd = kd;
    
    ctx->integral = 0.0f;
    return ESP_OK;
}
