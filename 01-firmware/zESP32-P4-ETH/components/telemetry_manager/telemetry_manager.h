#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle for a telemetry instance
 */
typedef void* telemetry_handle_t;

/**
 * @brief Create a new telemetry reporter
 * 
 * @param topic The MQTT topic to publish to
 * @param measurement The InfluxDB measurement name (e.g. "odometry", "system")
 * @param interval_ms The reporting interval in milliseconds
 * @return telemetry_handle_t Handle on success, NULL on failure
 */
telemetry_handle_t telemetry_create(const char *topic, const char *measurement, uint32_t interval_ms);

/**
 * @brief Set static tags for this telemetry instance (e.g. "sensor=battery")
 * 
 * @param handle Telemetry handle
 * @param tags Comma-separated tags or NULL to clear
 */
void telemetry_set_tags(telemetry_handle_t handle, const char *tags);

/**
 * @brief Add a float field to the next point
 * 
 * @param handle Telemetry handle
 * @param key Field key
 * @param value Field value
 */
void telemetry_add_float(telemetry_handle_t handle, const char *key, float value);

/**
 * @brief Add an integer field to the next point
 * 
 * @param handle Telemetry handle
 * @param key Field key
 * @param value Field value
 */
void telemetry_add_int(telemetry_handle_t handle, const char *key, int32_t value);

/**
 * @brief Add a boolean field to the next point
 * 
 * @param handle Telemetry handle
 * @param key Field key
 * @param value Field value
 */
void telemetry_add_bool(telemetry_handle_t handle, const char *key, bool value);

/**
 * @brief Commit all added fields as a single point in the batch buffer
 * 
 * @param handle Telemetry handle
 */
void telemetry_commit_point(telemetry_handle_t handle);

/**
 * @brief Destroy a telemetry reporter
 * 
 * @param handle Telemetry handle
 */
void telemetry_destroy(telemetry_handle_t handle);

#ifdef __cplusplus
}
#endif
