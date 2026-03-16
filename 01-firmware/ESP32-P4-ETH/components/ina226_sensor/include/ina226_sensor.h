/**
 * @file ina226_sensor.h
 * @brief High-precision Power Monitor Driver for INA226 (ESP32-P4)
 * 
 * Hardware assumptions:
 * - Shared I2C Bus with Audio Codec.
 * - Initialization depends on bus being already up.
 */

#ifndef INA226_SENSOR_H
#define INA226_SENSOR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the INA226 sensor.
 * 
 * @note Assumes I2C driver is already installed (usually by audio_player).
 * @return esp_err_t ESP_OK on success, ESP_FAIL if chip not found.
 */
esp_err_t ina226_sensor_init(void);

/**
 * @brief Reads the current bus voltage from the sensor.
 * 
 * @param[out] voltage_mv Pointer to store the voltage in millivolts.
 * @return esp_err_t ESP_OK on successful read.
 */
esp_err_t ina226_sensor_read_bus_voltage_mv(float *voltage_mv);

/**
 * @brief Reads the current flowing through the shunt resistor.
 * 
 * @param[out] current_ma Pointer to store the current in milliamperes.
 * @return esp_err_t ESP_OK on successful read.
 */
esp_err_t ina226_sensor_read_current_ma(float *current_ma);

/**
 * @brief Reads the calculated power consumption.
 * 
 * @param[out] power_mw Pointer to store the power in milliwatts.
 * @return esp_err_t ESP_OK on successful read.
 */
esp_err_t ina226_sensor_read_power_mw(float *power_mw);

#ifdef __cplusplus
}
#endif

#endif // INA226_SENSOR_H
