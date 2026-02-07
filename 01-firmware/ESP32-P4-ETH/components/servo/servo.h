/*
 * Servo component API (LEDC) - Header
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Servo configuration structure. Populate the fields and pass to
 * `servo_init()` to get a handle.
 */
typedef struct {
	int gpio_num;                   /**< GPIO number for servo signal */
	ledc_mode_t speed_mode;         /**< LEDC speed mode (LOW/HIGH) */
	ledc_timer_t timer_num;         /**< LEDC timer to use (LEDC_TIMER_0..) */
	ledc_channel_t channel;         /**< LEDC channel to use (LEDC_CHANNEL_0..) */
	ledc_timer_bit_t duty_resolution;/**< Resolution (e.g., LEDC_TIMER_13_BIT) */
	uint32_t freq_hz;               /**< PWM frequency (e.g., 50) */
	uint32_t min_pulse_us;          /**< Minimum servo pulse width in us (e.g., 500) */
	uint32_t max_pulse_us;          /**< Maximum servo pulse width in us (e.g., 2500) */
} servo_config_t;

/**
 * Opaque servo handle returned by `servo_init()`.
 */
typedef void *servo_handle_t;

/**
 * Initialize a servo instance with the provided configuration.
 * Returns a handle on success or NULL on failure.
 */
servo_handle_t servo_init(const servo_config_t *config);

/**
 * Deinitialize servo and free resources. Returns ESP_OK on success.
 */
esp_err_t servo_deinit(servo_handle_t handle);

/**
 * Configure pulse width bounds for an initialized servo.
 */
esp_err_t servo_set_bounds(servo_handle_t handle, uint32_t min_us, uint32_t max_us);

/**
 * Move servo to normalized position [0.0 .. 1.0].
 */
esp_err_t servo_move_normalized(servo_handle_t handle, float position);

/**
 * Move servo to specific pulse width (microseconds).
 */
esp_err_t servo_move_us(servo_handle_t handle, uint32_t pulse_us);

#ifdef __cplusplus
}
#endif
