#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the INA219 sensor on the I2C bus.
 * 
 * Uses configuration values defined in Kconfig:
 * - I2C Port, SDA, SCL pins
 * - Shunt resistor value (default 0.01 ohms)
 * - Maximum expected current
 * 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina219_sensor_init(void);

/**
 * @brief Read the bus voltage from the INA219.
 * 
 * @param[out] voltage_mv Pointer to store the voltage in millivolts.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina219_sensor_read_bus_voltage_mv(float *voltage_mv);

/**
 * @brief Read the current flowing through the shunt resistor.
 * 
 * @param[out] current_ma Pointer to store the current in milliamperes.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina219_sensor_read_current_ma(float *current_ma);

/**
 * @brief Read the power calculated by the INA219.
 * 
 * @param[out] power_mw Pointer to store the power in milliwatts.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina219_sensor_read_power_mw(float *power_mw);

#ifdef __cplusplus
}
#endif
