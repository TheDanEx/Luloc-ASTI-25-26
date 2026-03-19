#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CURVATURE_FF_TOPIC "robot/config/curvature"

/**
 * Register MQTT callback for curvature topic. Safe to call multiple times.
 */
esp_err_t curvature_feedforward_register_callback(void);

/**
 * Subscribe to curvature topic. Returns ESP_ERR_INVALID_STATE if MQTT is not connected.
 */
esp_err_t curvature_feedforward_subscribe(void);

/**
 * MQTT callback used by mqtt_custom_client.
 */
void curvature_feedforward_mqtt_callback(const char *topic, int topic_len, const char *data, int data_len);

/**
 * Access latest parsed curvature feedforward value.
 */
float curvature_feedforward_get_value(void);
uint32_t curvature_feedforward_get_timestamp_ms(void);
bool curvature_feedforward_has_value(void);

#ifdef __cplusplus
}
#endif
