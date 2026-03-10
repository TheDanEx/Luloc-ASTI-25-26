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
                                     float *out_pwm_duty);

/**
 * @brief Update PID constants in real-time.
 */
esp_err_t motor_velocity_ctrl_set_pid(motor_velocity_ctrl_handle_t handle, float kp, float ki, float kd);

/**
 * @brief Gets the current auto-sweeping target velocity for Live Calibration mode.
 * 
 * Automatically switches between CONFIG_VELOCITY_CTRL_SWEEP_SPEED_1 and SPEED_2
 * based on the elapsed time and CONFIG_VELOCITY_CTRL_SWEEP_TIME_MS.
 * 
 * @return float The current target speed (m/s).
 */
float motor_velocity_ctrl_get_sweep_target(void);

#ifdef __cplusplus
}
#endif
