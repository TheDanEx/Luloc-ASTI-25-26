#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/pcnt.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encoder configuration structure
 */
typedef struct {
    int pin_a;              // Phase A GPIO pin
    int pin_b;              // Phase B GPIO pin
    pcnt_unit_t pcnt_unit;  // PCNT unit (PCNT_UNIT_0 to PCNT_UNIT_3)
    int ppr;                // Pulses per revolution
} encoder_config_t;

/**
 * @brief Encoder state structure
 */
typedef struct {
    int32_t count;          // Current encoder count
    int32_t delta;          // Count delta since last read
    float speed_rpm;        // Speed in revolutions per minute
    float speed_rps;        // Speed in revolutions per second
    float speed_rad_s;      // Angular velocity in radians per second
    int32_t timestamp_ms;   // Timestamp of measurement in milliseconds
} encoder_state_t;

/**
 * @brief Encoder handle (opaque pointer)
 */
typedef void* encoder_handle_t;

/**
 * @brief Initialize encoder with configuration
 * 
 * @param config Pointer to encoder configuration
 * @return Encoder handle on success, NULL on failure
 */
encoder_handle_t encoder_init(const encoder_config_t *config);

/**
 * @brief Deinitialize encoder
 * 
 * @param handle Encoder handle
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t encoder_deinit(encoder_handle_t handle);

/**
 * @brief Read current encoder count
 * 
 * @param handle Encoder handle
 * @return Current count
 */
int32_t encoder_read_count(encoder_handle_t handle);

/**
 * @brief Get encoder count delta since last read
 * 
 * @param handle Encoder handle
 * @return Count difference
 */
int32_t encoder_get_delta(encoder_handle_t handle);

/**
 * @brief Get complete encoder state (updates all measurements)
 * 
 * This is the primary interface - call this to get all encoder data including
 * updated speed calculations
 * 
 * @param handle Encoder handle
 * @param state Pointer to encoder_state_t to fill
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle or state is NULL
 */
esp_err_t encoder_get_state(encoder_handle_t handle, encoder_state_t *state);

/**
 * @brief Get cached speed in RPM (last calculated value)
 * 
 * Call encoder_get_state() to update the cached value
 * 
 * @param handle Encoder handle
 * @return Speed in RPM
 */
float encoder_get_speed_rpm(encoder_handle_t handle);

/**
 * @brief Get cached speed in revolutions per second
 * 
 * @param handle Encoder handle
 * @return Speed in RPS
 */
float encoder_get_speed_rps(encoder_handle_t handle);

/**
 * @brief Get cached speed in radians per second
 * 
 * @param handle Encoder handle
 * @return Angular velocity in rad/s
 */
float encoder_get_speed_rad_s(encoder_handle_t handle);

/**
 * @brief Reset encoder count to zero
 * 
 * @param handle Encoder handle
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t encoder_reset_count(encoder_handle_t handle);

/**
 * @brief Get encoder configuration
 * 
 * @param handle Encoder handle
 * @param config Pointer to encoder_config_t to fill
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if handle or config is NULL
 */
esp_err_t encoder_get_config(encoder_handle_t handle, encoder_config_t *config);

/**
 * @brief Convert encoder state to JSON object
 * 
 * Creates a cJSON object containing all encoder state data
 * 
 * @param handle Encoder handle
 * @return cJSON object on success, NULL on failure
 *         Caller must free with cJSON_Delete()
 */
cJSON* encoder_state_to_json(encoder_handle_t handle);

/**
 * @brief Get encoder state as JSON string
 * 
 * Generates a formatted JSON string of encoder state
 * 
 * @param handle Encoder handle
 * @return JSON string pointer (must be freed with free())
 *         Returns NULL on failure
 */
char* encoder_state_to_json_string(encoder_handle_t handle);

#ifdef __cplusplus
}
#endif
