#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MQTT API Responder component.
 * Registers the callback to the incoming query topic.
 */
esp_err_t mqtt_api_responder_init(void);

/**
 * @brief Subscribe to the API GET topic.
 * Must be called when the MQTT Link is alive.
 */
esp_err_t mqtt_api_responder_subscribe(void);

#ifdef __cplusplus
}
#endif
