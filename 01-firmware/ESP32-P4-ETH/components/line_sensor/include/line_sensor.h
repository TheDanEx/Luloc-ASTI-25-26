/*
 * Line Sensor Array Component for ESP32-P4
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle for the line sensor instance (Class-like encapsulation)
typedef struct line_sensor_context* line_sensor_handle_t;

/**
 * @brief Configuration structure for the Line Sensor Array
 * 
 * Supports an arbitrary number of sensors by passing an array of ADC channels.
 */
typedef struct {
    int num_sensors;                    // Example: 8
    adc_unit_t adc_unit;                // Example: ADC_UNIT_1
    const adc_channel_t *adc_channels;  // Array of ADC channels to read
    const float *sensor_positions_m;    // Array of physical distances from center (meters)
    int oversample_count;               // If 0, uses menuconfig default
    int calibration_threshold;          // If 0, uses menuconfig default
    float detection_threshold;          // Normalized threshold (0.0 - 1.0). If 0.0, uses menuconfig %
} line_sensor_config_t;

/**
 * @brief Output data containing the processed line centroid and raw values
 */
typedef struct {
    float line_position_m;    // Centroid of the line in real metric units (meters)
    bool line_detected;       // True if a line is currently seen by ANY sensor
    uint16_t *raw_values;     // Array of raw reflectance values
    float *normalized_values; // Array of normalized values (0.0 to 1.0)
    bool *digital_states;     // Array of individual triggers (true if line is under sensor i)
} line_sensor_data_t;

/**
 * @brief Initialize the Line Sensor Array
 * 
 * @param config Pointer to the configuration struct
 * @return line_sensor_handle_t Handle to the initialized sensor, NULL on failure
 */
line_sensor_handle_t line_sensor_init(const line_sensor_config_t *config);

/**
 * @brief De-initialize the Line Sensor Array and free memory
 * 
 * @param handle Valid line sensor handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t line_sensor_deinit(line_sensor_handle_t handle);

/**
 * @brief Start background calibration process
 * 
 * Spawns a dedicated FreeRTOS task to sample sensors at high frequency.
 * @param handle Valid line sensor handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t line_sensor_calibration_start(line_sensor_handle_t handle);

/**
 * @brief Stop the calibration process
 * 
 * @param handle Valid line sensor handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t line_sensor_calibration_stop(line_sensor_handle_t handle);

/**
 * @brief Check if all sensors have seen both black and white distinctively
 * 
 * @param handle Valid line sensor handle
 * @return true if calibrated, false otherwise
 */
bool line_sensor_is_calibrated(line_sensor_handle_t handle);

/**
 * @brief Read the latest data (raw, normalized, and centroid)
 * 
 * @param handle Valid line sensor handle
 * @param out_data Pointer to the struct where data will be stored
 * @return esp_err_t ESP_OK on successful read
 */
esp_err_t line_sensor_read(line_sensor_handle_t handle, line_sensor_data_t *out_data);

/**
 * @brief Get only raw ADC values directly (oversampled average)
 * 
 * @param handle Valid line sensor handle
 * @param out_raw Array to store the raw values (must be sized >= num_sensors)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t line_sensor_read_raw(line_sensor_handle_t handle, uint16_t *out_raw);

/**
 * @brief Get only normalized values (0.0 to 1.0) based on calibration bounds
 * 
 * @param handle Valid line sensor handle
 * @param out_normalized Array to store values (must be sized >= num_sensors)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t line_sensor_read_normalized(line_sensor_handle_t handle, float *out_normalized);


#ifdef __cplusplus
}
#endif
