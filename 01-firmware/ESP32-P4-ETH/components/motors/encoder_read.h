#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encoder data structure
 */
typedef struct {
    int16_t count;           // Total encoder count
    int16_t delta;           // Change since last read
    int64_t elapsed_us;      // Elapsed time in microseconds since init
} encoder_data_t;

/**
 * @brief Initialize encoder with PCNT
 * 
 * Initializes GPIO, PCNT hardware, and timer reference
 * 
 * @return
 *          - ESP_OK on success
 *          - ESP_OK if already initialized
 */
esp_err_t encoder_init(void);

/**
 * @brief Get current encoder reading
 * 
 * @param data Pointer to encoder_data_t structure to fill
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if data is NULL
 *          - ESP_FAIL if encoder not initialized
 */
esp_err_t encoder_read(encoder_data_t *data);

/**
 * @brief Get encoder count
 * 
 * @return Current total encoder count
 */
int16_t encoder_get_count(void);

/**
 * @brief Get encoder delta (change since last read)
 * 
 * @return Delta count since last update
 */
int16_t encoder_get_delta(void);

/**
 * @brief Get elapsed time in microseconds
 * 
 * @return Elapsed time in microseconds since encoder initialization
 */
int64_t encoder_get_elapsed_us(void);

/**
 * @brief Get elapsed time in milliseconds
 * 
 * @return Elapsed time in milliseconds since encoder initialization
 */
uint32_t encoder_get_elapsed_ms(void);

/**
 * @brief Get elapsed time in seconds
 * 
 * @return Elapsed time in seconds since encoder initialization
 */
uint32_t encoder_get_elapsed_sec(void);

/**
 * @brief Clear encoder count
 * 
 * Resets PCNT counter and internal delta value
 */
void encoder_clear_count(void);

#ifdef __cplusplus
}
#endif
