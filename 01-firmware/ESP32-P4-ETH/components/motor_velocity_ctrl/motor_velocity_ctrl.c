#include "motor_velocity_ctrl.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =============================================================================
// Constants & Config
// =============================================================================
static const char *TAG = "VEL_CTRL";

struct motor_velocity_ctrl_context_t {
    motor_velocity_config_t config;
    
    // PID State
    float integral;
    float prev_filtered_speed;
    float current_filtered_speed;
    
    // Motion Profile State
    float ramped_target_speed;
};

// =============================================================================
// Helper Functions
// =============================================================================

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// =============================================================================
// Public API: Lifecycle
// =============================================================================

/**
 * Create a new velocity controller instance.
 * Initializes PID state and motion profiling buffers.
 */
esp_err_t motor_velocity_ctrl_create(const motor_velocity_config_t *config, motor_velocity_ctrl_handle_t *out_handle) {
    if (config == NULL || out_handle == NULL) return ESP_ERR_INVALID_ARG;

    struct motor_velocity_ctrl_context_t *ctx = calloc(1, sizeof(struct motor_velocity_ctrl_context_t));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for velocity controller");
        return ESP_ERR_NO_MEM;
    }

    ctx->config = *config;
    ctx->integral = 0.0f;
    ctx->prev_filtered_speed = 0.0f;
    ctx->current_filtered_speed = 0.0f;
    ctx->ramped_target_speed = 0.0f;

    *out_handle = ctx;
    ESP_LOGI(TAG, "Velocity controller created (P:%.2f I:%.2f D:%.2f)", 
             config->kp, config->ki, config->kd);
    return ESP_OK;
}

/**
 * Destroy the controller and free memory.
 */
esp_err_t motor_velocity_ctrl_destroy(motor_velocity_ctrl_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    free(handle);
    return ESP_OK;
}

// =============================================================================
// Public API: Control Logic
// =============================================================================

/**
 * Primary control loop update.
 * performs: 
 * 1. EMA Signal Filtering for noise reduction.
 * 2. Acceleration Slew Rate Limiting (Ramping).
 * 3. PID calculation using D-on-PV (Derivative on Process Variable) to avoid kicks.
 * 4. Feed-Forward mapping + Deadband compensation.
 * 5. Battery-aware PWM duty cycle conversion.
 */
esp_err_t motor_velocity_ctrl_update(motor_velocity_ctrl_handle_t handle, 
                                     const motor_velocity_input_t *input, 
                                     float delta_time_s,
                                     float *out_pwm_duty) {
    if (handle == NULL || input == NULL || out_pwm_duty == NULL) return ESP_ERR_INVALID_ARG;
    if (delta_time_s <= 0.0f) delta_time_s = 0.01f;

    struct motor_velocity_ctrl_context_t *ctx = handle;
    float battery_v = input->battery_mv / 1000.0f;
    if (battery_v < 1.0f) battery_v = ctx->config.max_battery_mv / 1000.0f; // Safety fallback

    // --- 1. Signal Filtering (EMA) ---
    // Smooth out encoder noise
    ctx->prev_filtered_speed = ctx->current_filtered_speed;
    ctx->current_filtered_speed = (ctx->config.ema_alpha * input->current_speed) + 
                                  ((1.0f - ctx->config.ema_alpha) * ctx->current_filtered_speed);

    // --- 2. Motion Generation (Acceleration Ramps) ---
    // Prevent sudden spikes in setpoint to protect mechanics
    float max_step = ctx->config.accel_limit_ms2 * delta_time_s;
    float speed_diff = input->target_speed - ctx->ramped_target_speed;
    ctx->ramped_target_speed += clamp(speed_diff, -max_step, max_step);

    // --- 3. PID Correction (D-on-PV) ---
    float error = ctx->ramped_target_speed - ctx->current_filtered_speed;
    
    // Integral with Anti-Windup (reset on zero target or stop accumulation on saturation)
    if (fabsf(ctx->ramped_target_speed) < 0.01f) {
        ctx->integral = 0.0f;
    } else {
        ctx->integral += error * delta_time_s;
        // Limit integral to avoid excessive windup (e.g. max 5V contribution)
        ctx->integral = clamp(ctx->integral, -5.0f, 5.0f);
    }

    // Derivative on Process Variable (D-on-PV) to avoid derivative kicks
    float derivative = (ctx->current_filtered_speed - ctx->prev_filtered_speed) / delta_time_s;

    float pid_voltage = (ctx->config.kp * error) + 
                        (ctx->config.ki * ctx->integral) - 
                        (ctx->config.kd * derivative);

    // --- 4. Deadband & Feed-Forward (Physics Base) ---
    float target_voltage_v = 0.0f;
    
    if (fabsf(ctx->ramped_target_speed) > 0.001f) {
        // Linear Feed-Forward: Speed to Voltage mapping
        float base_v = (ctx->ramped_target_speed / ctx->config.max_motor_speed) * 
                       (ctx->config.max_battery_mv / 1000.0f);
        
        // Add Deadband offset in the direction of movement
        float deadband_offset = (ctx->ramped_target_speed > 0) ? ctx->config.deadband_v : -ctx->config.deadband_v;
        
        target_voltage_v = base_v + deadband_offset + pid_voltage;
    }

    // --- 5. Battery Saturation & PWM Conversion ---
    // Rule: Output voltage cannot exceed available battery voltage
    float abs_target_v = fabsf(target_voltage_v);
    if (abs_target_v > battery_v) {
        target_voltage_v = (target_voltage_v > 0) ? battery_v : -battery_v;
    }

    // Convert to Duty Cycle: (V_desired / V_actual) * 100
    *out_pwm_duty = (target_voltage_v / battery_v) * 100.0f;
    *out_pwm_duty = clamp(*out_pwm_duty, -100.0f, 100.0f);

    return ESP_OK;
}

/**
 * Dynamic update of PID gains.
 * Useful for online tuning via MQTT/LiveTuning.
 */
esp_err_t motor_velocity_ctrl_set_pid(motor_velocity_ctrl_handle_t handle, float kp, float ki, float kd) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    
    struct motor_velocity_ctrl_context_t *ctx = handle;
    ctx->config.kp = kp;
    ctx->config.ki = ki;
    ctx->config.kd = kd;
    
    ctx->integral = 0.0f;
    return ESP_OK;
}

// =============================================================================
// Tools: Auto-Tuning / Sweeping
// =============================================================================

/**
 * Target generator for PID auto-tuning.
 * Cycles between two speeds configured via Kconfig to analyze step responses.
 */
float motor_velocity_ctrl_get_sweep_target(void) {
    static TickType_t sweep_last_toggle = 0;
    static bool sweep_phase_1 = true;
    static bool initialized = false;
    
    float sweep_speed_1 = atof(CONFIG_VELOCITY_CTRL_SWEEP_SPEED_1);
    float sweep_speed_2 = atof(CONFIG_VELOCITY_CTRL_SWEEP_SPEED_2);
    uint32_t sweep_time_ms = CONFIG_VELOCITY_CTRL_SWEEP_TIME_MS;

    if (!initialized) {
        sweep_last_toggle = xTaskGetTickCount();
        initialized = true;
    }

    TickType_t now = xTaskGetTickCount();
    if ((now - sweep_last_toggle) >= pdMS_TO_TICKS(sweep_time_ms)) {
        sweep_phase_1 = !sweep_phase_1;
        sweep_last_toggle = now;
        ESP_LOGI(TAG, "SWEEP: Altering target speed to %.2f m/s", 
                 sweep_phase_1 ? sweep_speed_1 : sweep_speed_2);
    }
    
    return sweep_phase_1 ? sweep_speed_1 : sweep_speed_2;
}
