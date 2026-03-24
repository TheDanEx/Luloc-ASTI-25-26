#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_SENSOR_COUNT 8

typedef enum {
    LINE_SENSOR_DIGITAL = 0,
    LINE_SENSOR_ANALOG
} line_sensor_type_t;

typedef struct {
    line_sensor_type_t type;
    int pin;
    uint32_t min_value;
    uint32_t max_value;
    float weight_mm; // Distance from center in mm
} line_sensor_config_t;

/**
 * @brief Initialize line sensor array
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t line_sensor_init(void);

/**
 * @brief Deinitialize and release resources
 */
void line_sensor_deinit(void);

/**
 * @brief Read raw values from all sensors
 * 
 * @param raw_values Array to store results (must be size LINE_SENSOR_COUNT)
 * @param num_samples Number of ADC samples for oversampling/averaging
 */
void line_sensor_read_raw(uint32_t *raw_values, uint32_t num_samples);

/**
 * @brief Read normalized values (0.0 to 1.0)
 * 
 * @param norm_values Array to store results (must be size LINE_SENSOR_COUNT)
 * @param raw_values_in Optional: provide previously read raw values. If NULL, reads internally.
 * @param num_samples Number of ADC samples if internal read is performed.
 */
void line_sensor_read_norm(float *norm_values, const uint32_t *raw_values_in, uint32_t num_samples);

/**
 * @brief Read binarized values (0 or 1)
 * 
 * @param bin_values Array to store results (must be size LINE_SENSOR_COUNT)
 * @param norm_values_in Optional: provide previously read normalized values. If NULL, reads internally.
 * @param num_samples Number of ADC samples if internal read is performed.
 */
void line_sensor_read_bin(uint8_t *bin_values, const float *norm_values_in, uint32_t num_samples);

/**
 * @brief Calculate position of line in mm relative to center
 * 
 * @param norm_values_in Optional: provide previously read normalized values. If NULL, reads internally.
 * @param num_samples Number of ADC samples if internal read is performed.
 * @return float Position in mm. 0 is center.
 */
float line_sensor_read_line_position(const float *norm_values_in, uint32_t num_samples);

/**
 * @brief Update calibration values for a specific sensor
 * 
 * @param index Sensor index (0 to 7)
 * @param min_val New minimum value
 * @param max_val New maximum value
 */
void line_sensor_set_calibration(uint8_t index, uint32_t min_val, uint32_t max_val);

/**
 * @brief Get current calibration values for a sensor
 */
void line_sensor_get_calibration(uint8_t index, uint32_t *min_val, uint32_t *max_val);

#ifdef __cplusplus
}
#endif
