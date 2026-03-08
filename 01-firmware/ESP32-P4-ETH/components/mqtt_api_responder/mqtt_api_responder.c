#include "mqtt_api_responder.h"
#include "esp_log.h"
#include "cJSON.h"
#include "mqtt_custom_client.h"
#include "shared_memory.h"
#include "test_sensor.h"
#include <string.h>

static const char *TAG = "API_RESPONDER";

#ifndef CONFIG_MQTT_API_GET_TOPIC
#define CONFIG_MQTT_API_GET_TOPIC "robot/api/get"
#endif

#ifndef CONFIG_MQTT_API_RESP_TOPIC
#define CONFIG_MQTT_API_RESP_TOPIC "robot/api/response"
#endif

// =============================================================================
// Resource Handlers
// =============================================================================

static void handle_resource_battery(cJSON *response_root) {
    shared_memory_t* shm = shared_memory_get();
    float bat_mv = 0.0f;
    float current_ma = 0.0f;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bat_mv = shm->sensors.battery_voltage;
        current_ma = shm->sensors.motor_current;
        xSemaphoreGive(shm->mutex);
    }
    
    cJSON_AddNumberToObject(response_root, "battery_mv", bat_mv);
    cJSON_AddNumberToObject(response_root, "motor_current_ma", current_ma);
}

static void handle_resource_encoder(cJSON *response_root) {
    shared_memory_t* shm = shared_memory_get();
    float speed = 0.0f;
    int32_t ticks = 0;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        speed = shm->sensors.motor_speed;
        ticks = shm->sensors.encoder_count;
        xSemaphoreGive(shm->mutex);
    }
    
    cJSON_AddNumberToObject(response_root, "speed_ms", speed);
    cJSON_AddNumberToObject(response_root, "encoder_ticks", ticks);
}

static void handle_resource_uptime(cJSON *response_root) {
    test_sensor_data_t uptime_data;
    if (test_sensor_read(&uptime_data) == ESP_OK) {
        cJSON_AddNumberToObject(response_root, "uptime_sec", uptime_data.uptime_sec);
        cJSON_AddNumberToObject(response_root, "uptime_ms", uptime_data.uptime_ms);
    }
}

// =============================================================================
// MQTT Callback
// =============================================================================

static void api_mqtt_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (topic_len <= 0 || data_len <= 0 || data_len > 512) return;
    if (strncmp(topic, CONFIG_MQTT_API_GET_TOPIC, topic_len) != 0) return;

    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) {
        ESP_LOGE(TAG, "Invalid JSON Request");
        return;
    }

    cJSON *resource_item = cJSON_GetObjectItem(root, "resource");
    if (!cJSON_IsString(resource_item) || (resource_item->valuestring == NULL)) {
        cJSON_Delete(root);
        return;
    }

    const char *resource = resource_item->valuestring;
    ESP_LOGI(TAG, "API Request for resource: %s", resource);

    // Build the dynamic response
    cJSON *response_root = cJSON_CreateObject();
    cJSON_AddStringToObject(response_root, "resource", resource);

    if (strcmp(resource, "battery") == 0) {
        handle_resource_battery(response_root);
    } 
    else if (strcmp(resource, "encoder") == 0) {
        handle_resource_encoder(response_root);
    }
    else if (strcmp(resource, "uptime") == 0) {
        handle_resource_uptime(response_root);
    }
    else if (strcmp(resource, "all") == 0) {
        handle_resource_battery(response_root);
        handle_resource_encoder(response_root);
        handle_resource_uptime(response_root);
    }
    else {
        cJSON_AddStringToObject(response_root, "error", "Unknown Resource");
    }

    // Stringify and publish
    char *json_str = cJSON_PrintUnformatted(response_root);
    if (json_str != NULL) {
        mqtt_custom_client_publish(CONFIG_MQTT_API_RESP_TOPIC, json_str, 0, 0, 0); // QoS0 is fine for polls
        free(json_str);
    }

    // Clean memory
    cJSON_Delete(response_root);
    cJSON_Delete(root);
}

// =============================================================================
// Init
// =============================================================================

esp_err_t mqtt_api_responder_init(void) {
    return mqtt_custom_client_register_topic_callback(CONFIG_MQTT_API_GET_TOPIC, api_mqtt_callback);
}

esp_err_t mqtt_api_responder_subscribe(void) {
    if (!mqtt_custom_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    int res = mqtt_custom_client_subscribe(CONFIG_MQTT_API_GET_TOPIC, 0);
    return (res < 0) ? ESP_FAIL : ESP_OK;
}
