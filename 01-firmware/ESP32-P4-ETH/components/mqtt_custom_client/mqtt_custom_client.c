/*
 * SPDX-License-Identifier: MIT
 * 
 * Custom MQTT Client Component
 * Wraps ESP-IDF MQTT component with simplified API
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "mqtt_custom_client.h"

static const char *TAG = "mqtt_custom_client";

// Global MQTT client handle
static esp_mqtt_client_handle_t mqtt_client_handle = NULL;
static bool mqtt_connected = false;

// Callback storage for incoming messages
#define MQTT_CALLBACK_MAX 10
typedef struct {
    char topic[64];
    mqtt_message_callback_t callback;
} mqtt_topic_callback_t;

static mqtt_topic_callback_t g_topic_callbacks[MQTT_CALLBACK_MAX] = {0};

/**
 * @brief Internal MQTT event handler
 * 
 * Processes all MQTT events from the ESP-IDF mqtt component
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        mqtt_connected = true;
        ESP_LOGD(TAG, "MQTT Connected to broker %s", CONFIG_BROKER_URL);
        // Publish startup event when connected
        esp_mqtt_client_publish(mqtt_client_handle, "robot/events", "{\"event\":\"connected\",\"msg\":\"ESP32 MQTT connection started\"}", 0, 1, 0);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        mqtt_connected = false;
        // Only print if we were previously connected or if it's a distinct event
        ESP_LOGW(TAG, "MQTT Disconnected - retrying...");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG, "Subscribed, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGD(TAG, "Unsubscribed, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "Published, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "Data - Topic: %.*s", event->topic_len, event->topic);
        
        // Check registered callbacks
        bool callback_found = false;
        for (int i = 0; i < MQTT_CALLBACK_MAX; i++) {
            if (g_topic_callbacks[i].callback != NULL) {
                // Check if this message matches the registered topic
                size_t registered_topic_len = strlen(g_topic_callbacks[i].topic);
                if (registered_topic_len == event->topic_len && 
                    strncmp(g_topic_callbacks[i].topic, event->topic, event->topic_len) == 0) {
                    g_topic_callbacks[i].callback(event->topic, event->topic_len, 
                                                   event->data, event->data_len);
                    callback_found = true;
                }
            }
        }
        if (!callback_found) {
            ESP_LOGD(TAG, "No callback found for topic: %.*s", event->topic_len, event->topic);
        }
        break;
        
    case MQTT_EVENT_ERROR:
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGW(TAG, "MQTT Connection Failed: %s", strerror(event->error_handle->esp_transport_sock_errno));
        } else {
             ESP_LOGD(TAG, "MQTT Error event");
        }
        break;
        
    default:
        break;
    }
}

/**
 * @brief Initialize and start MQTT client
 */
esp_err_t mqtt_custom_client_init(void)
{
    if (mqtt_client_handle != NULL) {
        ESP_LOGW(TAG, "MQTT client already initialized");
        return ESP_OK;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    mqtt_client_handle = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    // Suppress verbose logs from underlying MQTT components
    esp_log_level_set("mqtt_client", ESP_LOG_NONE);
    esp_log_level_set("esp-tls", ESP_LOG_NONE);
    esp_log_level_set("transport_base", ESP_LOG_NONE);
    esp_log_level_set("transport", ESP_LOG_NONE);
    esp_log_level_set("TRANS_TCP", ESP_LOG_NONE);

    // Register event handler
    esp_mqtt_client_register_event(mqtt_client_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // Start MQTT client
    esp_err_t ret = esp_mqtt_client_start(mqtt_client_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: 0x%x", ret);
        return ret;
    }

    ESP_LOGD(TAG, "MQTT client initialized and started");
    return ESP_OK;
}

/**
 * @brief Publish message to topic
 */
int mqtt_custom_client_publish(const char *topic, const char *data, int len, int qos, int retain)
{
    if (mqtt_client_handle == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return -1;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client_handle, topic, data, len, qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to topic %s", topic);
        return -1;
    }

    ESP_LOGD(TAG, "Published to %s, msg_id=%d", topic, msg_id);
    return msg_id;
}

/**
 * @brief Subscribe to topic
 */
int mqtt_custom_client_subscribe(const char *topic, int qos)
{
    if (mqtt_client_handle == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return -1;
    }

    int msg_id = esp_mqtt_client_subscribe(mqtt_client_handle, topic, qos);
    if (msg_id < 0) {
        // Only log warning if not connected, otherwise error 
        if (!mqtt_custom_client_is_connected()) {
             // Suppress noise, common during startup before connection
        } else {
             ESP_LOGW(TAG, "Failed to subscribe to topic %s", topic);
        }
        return -1;
    }

    ESP_LOGD(TAG, "Subscribed to %s, msg_id=%d", topic, msg_id);
    return msg_id;
}

/**
 * @brief Unsubscribe from topic
 */
int mqtt_custom_client_unsubscribe(const char *topic)
{
    if (mqtt_client_handle == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return -1;
    }

    int msg_id = esp_mqtt_client_unsubscribe(mqtt_client_handle, topic);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to unsubscribe from topic %s", topic);
        return -1;
    }

    ESP_LOGD(TAG, "Unsubscribed from %s, msg_id=%d", topic, msg_id);
    return msg_id;
}

/**
 * @brief Check if MQTT client is connected
 */
bool mqtt_custom_client_is_connected(void)
{
    return mqtt_connected;
}

#include <stdarg.h>

int mqtt_custom_client_log(const char *level, const char *fmt, ...)
{
    if (mqtt_client_handle == NULL || !mqtt_connected || level == NULL || fmt == NULL) {
        return -1;
    }

    char topic[64];
    snprintf(topic, sizeof(topic), "robot/logs/%s", level);

    char message[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    if (len < 0) return -1;

    return mqtt_custom_client_publish(topic, message, 0, 0, 0);
}

int mqtt_custom_client_debug(const char *fmt, ...)
{
    if (mqtt_client_handle == NULL || !mqtt_connected || fmt == NULL) {
        return -1;
    }

    char message[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    if (len < 0) return -1;

    return mqtt_custom_client_publish("robot/debug", message, 0, 0, 0);
}

/**
 * @brief Register callback for specific topic
 */
esp_err_t mqtt_custom_client_register_topic_callback(const char *topic, mqtt_message_callback_t callback)
{
    if (topic == NULL || callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find free slot
    for (int i = 0; i < MQTT_CALLBACK_MAX; i++) {
        if (g_topic_callbacks[i].callback == NULL) {
            strncpy(g_topic_callbacks[i].topic, topic, sizeof(g_topic_callbacks[i].topic) - 1);
            g_topic_callbacks[i].topic[sizeof(g_topic_callbacks[i].topic) - 1] = '\0';  // Ensure null termination
            g_topic_callbacks[i].callback = callback;
            ESP_LOGD(TAG, "Registered callback for topic: %s", topic);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "No free callback slots (max: %d)", MQTT_CALLBACK_MAX);
    return ESP_ERR_NO_MEM;
}
