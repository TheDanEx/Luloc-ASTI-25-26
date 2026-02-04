#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encoder Sensor configuration structure
 */
typedef struct {
    int pin_a;                  // Phase A GPIO pin
    int pin_b;                  // Phase B GPIO pin
    int ppr;                    // Pulses per Revolution (of the motor shaft usually)
    float gear_ratio;           // Gear reduction ratio (e.g. 1.0 for direct, 50.0 for 50:1)
    float wheel_diameter_m;     // Wheel diameter in meters
    bool reverse_direction;     // Reverse counting direction
} encoder_sensor_config_t;

/**
 * @brief Encoder Sensor handle (opaque pointer)
 */
typedef void* encoder_sensor_handle_t;

/**
 * @brief Initialize encoder sensor
 * 
 * @param config Pointer to configuration structure
 * @return Encoder handle on success, NULL on failure
 */
encoder_sensor_handle_t encoder_sensor_init(const encoder_sensor_config_t *config);

/**
 * @brief Deinitialize encoder sensor
 * 
 * @param handle Encoder handle
 * @return ESP_OK on success
 */
esp_err_t encoder_sensor_deinit(encoder_sensor_handle_t handle);

/**
 * @brief Get current linear speed in meters per second
 * 
 * @param handle Encoder handle
 * @return Speed in m/s
 */
float encoder_sensor_get_speed(encoder_sensor_handle_t handle);

/**
 * @brief Get total distance traveled in meters
 * 
 * @param handle Encoder handle
 * @return Distance in meters
 */
float encoder_sensor_get_distance(encoder_sensor_handle_t handle);

/**
 * @brief Reset distance counter to zero
 * 
 * @param handle Encoder handle
 * @return ESP_OK on success
 */
esp_err_t encoder_sensor_reset_distance(encoder_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif
