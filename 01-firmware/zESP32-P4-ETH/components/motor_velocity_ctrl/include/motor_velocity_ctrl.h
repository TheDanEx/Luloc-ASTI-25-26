#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motor_velocity_ctrl_context_t* motor_velocity_ctrl_handle_t;

/**
 * @brief Configuration struct for the motor velocity controller.
 * Can be populated via Kconfig or manually defined.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_battery_mv;    // e.g. 16800.0f
    float max_motor_speed;   // e.g. 1.5f (m/s)
    float deadband_v;        // Minimum voltage to overcome friction (e.g. 1.2f)
    float accel_limit_ms2;   // Max acceleration (e.g. 2.5f m/s^2)
    float ema_alpha;         // Encoder filtering alpha (0.0 - 1.0)
} motor_velocity_config_t;

/**
 * @brief Input parameters injected per tick.
 */
typedef struct {
    float target_speed;      // The desired linear speed (m/s, negative for reverse)
    float current_speed;     // The actual linear speed from the encoder (m/s)
    float battery_mv;        // Instantaneous battery voltage from INA219
} motor_velocity_input_t;

/**
 * @brief Diagnostic data for PID analysis.
 */
typedef struct {
    float target_ramped;    // Target after acceleration ramping (m/s)
    float error;            // P contribution base (m/s)
    
    // Voltage contributions (V)
    float feed_forward_v;   // Voltage from Feed-Forward (model-based)
    float p_v;             // Voltage from Proportional term
    float i_v;             // Voltage from Integral term
    float d_v;             // Voltage from Derivative term
    
    float final_v;          // Total target voltage (sum, saturated)
    float pwm_duty;         // Resulting PWM % (-100 to 100)
} motor_velocity_diag_t;

/**
 * @brief Create a velocity controller instance for a single motor.
 * @param config Pointer to configuration parameters.
 * @param out_handle Pointer to store the new handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t motor_velocity_ctrl_create(const motor_velocity_config_t *config, motor_velocity_ctrl_handle_t *out_handle);

/**
 * @brief Destroy a velocity controller instance.
 */
esp_err_t motor_velocity_ctrl_destroy(motor_velocity_ctrl_handle_t handle);

/**
 * @brief Update the controller and calculate the required PWM duty cycle.
 * 
 * Computes a Feed-Forward prediction based on battery voltage, then 
 * adds a PID correction based on the RPM error. All values are scaled
 * cleanly.
 * 
 * @param handle Controller instance.
 * @param input Sensor and target data.
 * @param delta_time_s Elapsed time in seconds.
 * @param out_pwm_duty Output PWM duty cycle (-100.0 to 100.0).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t motor_velocity_ctrl_update(motor_velocity_ctrl_handle_t handle, 
                                     const motor_velocity_input_t *input, 
                                     float delta_time_s,
                                     float *out_pwm_duty,
                                     motor_velocity_diag_t *out_diag);

/**
 * @brief Update PID constants in real-time.
 */
esp_err_t motor_velocity_ctrl_set_pid(motor_velocity_ctrl_handle_t handle, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif
