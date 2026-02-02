#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Test sensor data structure
 */
typedef struct {
    uint32_t uptime_ms;      // Uptime in milliseconds
    uint32_t uptime_sec;     // Uptime in seconds
    uint32_t uptime_min;     // Uptime in minutes
} test_sensor_data_t;

/**
 * @brief Initialize test sensor
 * 
 * Simply tracks ESP32 uptime using esp_timer_get_time()
 * 
 * @return
 *          - ESP_OK on success
 *          - ESP_FAIL on failure
 */
esp_err_t test_sensor_init(void);

/**
 * @brief Get current sensor reading (uptime)
 * 
 * @param data Pointer to test_sensor_data_t structure to fill
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG if data is NULL
 */
esp_err_t test_sensor_read(test_sensor_data_t *data);

/**
 * @brief Get uptime in milliseconds
 * 
 * @return Uptime in milliseconds since boot
 */
uint32_t test_sensor_get_uptime_ms(void);

/**
 * @brief Get uptime in seconds
 * 
 * @return Uptime in seconds since boot
 */
uint32_t test_sensor_get_uptime_sec(void);

/**
 * @brief Get uptime as formatted string
 * 
 * @param buffer Buffer to store the formatted string
 * @param buffer_size Size of buffer
 * @return Pointer to buffer with formatted uptime (e.g., "1h 23m 45s")
 */
const char *test_sensor_get_uptime_str(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif
