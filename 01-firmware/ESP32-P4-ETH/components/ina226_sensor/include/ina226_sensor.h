/**
 * @file ina226_sensor.h
 * @brief High-precision Power Monitor Driver for INA226.
 */

#ifndef INA226_SENSOR_H
#define INA226_SENSOR_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Types & Structures
// =============================================================================

/**
 * @brief Unified data structure for power system metrics.
 */
typedef struct {
    float voltage_mv;
    float current_ma;
    float power_mw;
} ina_data_t;

// =============================================================================
// Public API
// =============================================================================

/**
 * @brief Initialize the INA226 sensor hardware.
 */
esp_err_t ina_init(void);

/**
 * @brief Read individual power system metrics.
 */
esp_err_t ina_read_voltage(float *voltage_mv);
esp_err_t ina_read_current(float *current_ma);
esp_err_t ina_read_power(float *power_mw);

/**
 * @brief Capture all system metrics and optionally process audible alerts.
 * 
 * @param[out] data Summary structure with all readings.
 * @param check_alerts If true, executes threshold logic and plays audio.
 */
esp_err_t ina_read(ina_data_t *data, bool check_alerts);

/**
 * @brief Internal threshold monitor and audio alert trigger.
 */
esp_err_t ina_check_alerts(void);

#ifdef __cplusplus
}
#endif

#endif // INA226_SENSOR_H
