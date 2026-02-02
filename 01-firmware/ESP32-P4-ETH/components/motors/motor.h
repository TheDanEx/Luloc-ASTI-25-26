#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/pcnt.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor configuration structure
 */
typedef struct {
    int encoder_pin_a;      // Encoder Phase A GPIO pin
    int encoder_pin_b;      // Encoder Phase B GPIO pin
    pcnt_unit_t pcnt_unit;  // PCNT unit (PCNT_UNIT_0 to PCNT_UNIT_3)
    int ppr;                // Pulses Per Revolution
} motor_config_t;

/**
 * @brief Motor handle (opaque pointer)
 */
typedef void* motor_handle_t;

/**
 * @brief Motor state structure for data output
 */
typedef struct {
    int32_t encoder_count;  // Current encoder count
    int32_t count_delta;    // Count change since last read
    float speed_rpm;        // Calculated speed in RPM
    float speed_rps;        // Speed in revolutions per second
    float speed_rad_s;      // Angular velocity in rad/s
    int32_t timestamp_ms;   // Timestamp of measurement
} motor_state_t;

/**
 * @brief Initialize motor with encoder pins and PCNT unit
 * 
 * @param config Pointer to motor configuration structure
 * @return Motor handle on success, NULL on failure
 */
motor_handle_t motor_init(const motor_config_t *config);

/**
 * @brief Deinitialize motor and cleanup resources
 * 
 * @param handle Motor handle
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t motor_deinit(motor_handle_t handle);

/**
 * @brief Read current encoder count
 * 
 * @param handle Motor handle
 * @return Current encoder count
 */
int32_t motor_get_encoder_count(motor_handle_t handle);

/**
 * @brief Reset encoder count to zero
 * 
 * @param handle Motor handle
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t motor_reset_encoder(motor_handle_t handle);

/**
 * @brief Get motor state including speed calculations
 * 
 * @param handle Motor handle
 * @param state Pointer to motor_state_t structure to fill
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle or state is NULL
 */
esp_err_t motor_get_state(motor_handle_t handle, motor_state_t *state);

/**
 * @brief Get encoder count delta since last call
 * 
 * @param handle Motor handle
 * @return Count difference since last read
 */
int32_t motor_get_count_delta(motor_handle_t handle);

/**
 * @brief Calculate and get current motor speed in RPM
 * 
 * @param handle Motor handle
 * @return Speed in RPM
 */
float motor_get_speed_rpm(motor_handle_t handle);

/**
 * @brief Generate JSON representation of motor state
 * 
 * @param handle Motor handle
 * @return Pointer to cJSON object, caller must free with cJSON_Delete
 */
cJSON* motor_state_to_json(motor_handle_t handle);

/**
 * @brief Get motor configuration
 * 
 * @param handle Motor handle
 * @param config Pointer to motor_config_t structure to fill
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle or config is NULL
 */
esp_err_t motor_get_config(motor_handle_t handle, motor_config_t *config);

#ifdef __cplusplus
}
#endif
