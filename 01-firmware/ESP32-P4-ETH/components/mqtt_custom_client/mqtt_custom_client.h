#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback function for incoming MQTT messages
 * 
 * Called when a message is received on a subscribed topic
 * @param topic Topic name
 * @param topic_len Length of topic
 * @param data Message data
 * @param data_len Length of message data
 */
typedef void (*mqtt_message_callback_t)(const char *topic, int topic_len, const char *data, int data_len);

/**
 * @brief Initialize and start MQTT client
 * 
 * Connects to the MQTT broker specified in CONFIG_BROKER_URL
 * Must be called after esp_event_loop_create_default() and networking is initialized
 * 
 * @return
 *          - ESP_OK on success
 *          - ESP_FAIL on initialization failure
 */
esp_err_t mqtt_custom_client_init(void);

/**
 * @brief Publish a message to an MQTT topic
 * 
 * @param topic Topic name (e.g., "robot/events")
 * @param data Message data (pointer to data)
 * @param len Data length (pass 0 for NULL-terminated string)
 * @param qos Quality of Service (0 or 1)
 * @param retain Retain flag
 * @return Message ID if successful, -1 on failure
 */
int mqtt_custom_client_publish(const char *topic, const char *data, int len, int qos, int retain);

/**
 * @brief Subscribe to an MQTT topic
 * 
 * @param topic Topic name (e.g., "robot/cmd")
 * @param qos Quality of Service (0 or 1)
 * @return Message ID if successful, -1 on failure
 */
int mqtt_custom_client_subscribe(const char *topic, int qos);

/**
 * @brief Unsubscribe from an MQTT topic
 * 
 * @param topic Topic name
 * @return Message ID if successful, -1 on failure
 */
int mqtt_custom_client_unsubscribe(const char *topic);

/**
 * @brief Check if MQTT client is connected
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_custom_client_is_connected(void);

/**
 * @brief Register callback for incoming messages on specific topic
 * 
 * @param topic Topic to monitor (e.g., "robot/cmd")
 * @param callback Function to call when message arrives
 * @return ESP_OK on success
 */
esp_err_t mqtt_custom_client_register_topic_callback(const char *topic, mqtt_message_callback_t callback);

#ifdef __cplusplus
}
#endif
