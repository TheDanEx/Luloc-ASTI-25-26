#include "mqtt_api_responder.h"
#include "esp_log.h"
#include "cJSON.h"
#include "mqtt_custom_client.h"
#include "shared_memory.h"
#include "test_sensor.h"
#include "audio_player.h"
#include "state_machine.h"
#include "ptp_client.h"
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdio.h>

static const char *TAG = "API_RESPONDER";

#ifndef CONFIG_MQTT_API_REQ_TOPIC
#define CONFIG_MQTT_API_REQ_TOPIC "robot/api/request"
#endif

#ifndef CONFIG_MQTT_API_RESP_TOPIC
#define CONFIG_MQTT_API_RESP_TOPIC "robot/api/response"
#endif

// =============================================================================
// Resource Handlers
// =============================================================================

/**
 * Collect battery and current data from shared memory
 */
static void handle_resource_battery(cJSON *response_root) {
    shared_memory_t* shm = shared_memory_get();
    float bat_mv = 0.0f;
    float current_ma = 0.0f;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bat_mv = shm->sensors.battery_voltage;
        current_ma = shm->sensors.robot_current;
        xSemaphoreGive(shm->mutex);
    }
    
    cJSON_AddNumberToObject(response_root, "battery_mv", bat_mv);
    cJSON_AddNumberToObject(response_root, "robot_current_ma", current_ma);
}

/**
 * Collect motor speeds and encoder ticks from shared memory
 */
static void handle_resource_encoder(cJSON *response_root) {
    shared_memory_t* shm = shared_memory_get();
    float speed_l = 0.0f, speed_r = 0.0f;
    int32_t ticks_l = 0, ticks_r = 0;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        speed_l = shm->sensors.motor_speed_left;
        speed_r = shm->sensors.motor_speed_right;
        ticks_l = shm->sensors.encoder_count_left;
        ticks_r = shm->sensors.encoder_count_right;
        xSemaphoreGive(shm->mutex);
    }
    
    cJSON_AddNumberToObject(response_root, "speed_l_ms", speed_l);
    cJSON_AddNumberToObject(response_root, "speed_r_ms", speed_r);
    cJSON_AddNumberToObject(response_root, "ticks_l", ticks_l);
    cJSON_AddNumberToObject(response_root, "ticks_r", ticks_r);
}

/**
 * Collect system uptime and timer data
 */
static void handle_resource_uptime(cJSON *response_root) {
    test_sensor_data_t uptime_data;
    if (test_sensor_read(&uptime_data) == ESP_OK) {
        cJSON_AddNumberToObject(response_root, "uptime_sec", uptime_data.uptime_sec);
        cJSON_AddNumberToObject(response_root, "uptime_us", (double)esp_timer_get_time()); 
    }
}

/**
 * Collect current robot state and mode
 */
static void handle_resource_status(cJSON *response_root) {
    robot_state_context_t* ctx = state_machine_get_context();
    cJSON_AddNumberToObject(response_root, "state_id", ctx->current_state);
    cJSON_AddStringToObject(response_root, "state_str", get_state_name(ctx->current_state));
    cJSON_AddNumberToObject(response_root, "mode_id", ctx->current_mode);
    cJSON_AddStringToObject(response_root, "mode_str", get_mode_name(ctx->current_mode));
}

// =============================================================================
// Action Handlers (SET)
// =============================================================================

/**
 * Execute mode change request and publish ILP event
 */
static void handle_action_set_mode(cJSON *root, cJSON *response_root) {
    cJSON *mode_id_item = cJSON_GetObjectItem(root, "mode_id");
    cJSON *force_item = cJSON_GetObjectItem(root, "force");
    bool force = false;

    if (cJSON_IsBool(force_item)) {
        force = cJSON_IsTrue(force_item);
    }

    if (cJSON_IsNumber(mode_id_item)) {
        int mode_id = mode_id_item->valueint;
        if (mode_id >= MODE_NONE && mode_id < MODE_COUNT) {
            
            if (state_machine_request_mode((robot_mode_t)mode_id, force)) {
                cJSON_AddStringToObject(response_root, "status", "success");
                cJSON_AddStringToObject(response_root, "message", "Mode changed");
                
                // Publish the asynchronous event using ILP
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                int64_t timestamp_ns = (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;

#ifdef CONFIG_TELEMETRY_ROBOT_NAME
                const char *robot_name = CONFIG_TELEMETRY_ROBOT_NAME;
#else
                const char *robot_name = "unknown";
#endif

                char ilp_payload[192];
                snprintf(ilp_payload, sizeof(ilp_payload), 
                        "events,type=MODE_CHANGE,robot=%s mode=%di,mode_str=\"%s\" %lld", 
                        robot_name, mode_id, get_mode_name((robot_mode_t)mode_id), timestamp_ns);
                
                mqtt_custom_client_publish("robot/events", ilp_payload, 0, 1, 0);
            } else {
                cJSON_AddStringToObject(response_root, "status", "error");
                cJSON_AddStringToObject(response_root, "message", "Mode change rejected");
                
                // Also publish error event in ILP
                struct timespec ts_err;
                clock_gettime(CLOCK_REALTIME, &ts_err);
                int64_t ts_err_ns = (int64_t)ts_err.tv_sec * 1000000000LL + (int64_t)ts_err.tv_nsec;
                
                char err_payload[128];
                snprintf(err_payload, sizeof(err_payload), "events,type=ERROR,robot=unknown error=\"MODE_REJECTED\" %lld", ts_err_ns);
                mqtt_custom_client_publish("robot/events", err_payload, 0, 1, 0);
            }
        } else {
            cJSON_AddStringToObject(response_root, "status", "error");
            cJSON_AddStringToObject(response_root, "message", "Invalid mode_id");
        }
    } else {
        cJSON_AddStringToObject(response_root, "status", "error");
        cJSON_AddStringToObject(response_root, "message", "Missing mode_id");
    }
}

/**
 * Handle audio playback requests
 */
static void handle_action_play_sound(cJSON *root, cJSON *response_root) {
    cJSON *sound_id_item = cJSON_GetObjectItem(root, "sound_id");
    cJSON *volume_item = cJSON_GetObjectItem(root, "volume");

    if (cJSON_IsNumber(sound_id_item)) {
        int sound_id = sound_id_item->valueint;
        if (sound_id >= 0 && sound_id < SOUND_MAX) {
            
            if (cJSON_IsNumber(volume_item)) {
                audio_player_play_vol((audio_sound_t)sound_id, (uint8_t)volume_item->valueint);
            } else {
                audio_player_play((audio_sound_t)sound_id);
            }
            
            cJSON_AddStringToObject(response_root, "status", "success");
            cJSON_AddStringToObject(response_root, "message", "Sound queued");
        } else {
            cJSON_AddStringToObject(response_root, "status", "error");
            cJSON_AddStringToObject(response_root, "message", "Invalid sound_id");
        }
    } else {
        cJSON_AddStringToObject(response_root, "status", "error");
        cJSON_AddStringToObject(response_root, "message", "Missing sound_id");
    }
}

/**
 * Update the shared motor calibration mask
 */
static void handle_action_set_cal_mask(cJSON *root, cJSON *response_root) {
    cJSON *mask_item = cJSON_GetObjectItem(root, "mask");
    if (cJSON_IsNumber(mask_item)) {
        uint8_t mask = (uint8_t)mask_item->valueint;
        shared_memory_t* shm = shared_memory_get();
        if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            shm->calibration_motor_mask = mask;
            xSemaphoreGive(shm->mutex);
            cJSON_AddStringToObject(response_root, "status", "success");
            cJSON_AddStringToObject(response_root, "message", "Calibration mask updated");
        } else {
            cJSON_AddStringToObject(response_root, "status", "error");
            cJSON_AddStringToObject(response_root, "message", "Shared memory timeout");
        }
    } else {
        cJSON_AddStringToObject(response_root, "status", "error");
        cJSON_AddStringToObject(response_root, "message", "Missing or invalid mask");
    }
}

/**
 * Force PTP/SNTP Time Synchronization
 */
static void handle_action_sync_time(cJSON *root, cJSON *response_root) {
    ptp_client_force_sync();
    cJSON_AddStringToObject(response_root, "status", "success");
    cJSON_AddStringToObject(response_root, "message", "PTP Sync Forced");
}

// =============================================================================
// MQTT Callback
// =============================================================================

/**
 * Main dispatcher for unified API requests
 */
static void api_mqtt_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (topic_len <= 0 || data_len <= 0 || data_len > 1024) return;
    
    // Check if it's our unified request topic
    if (strncmp(topic, CONFIG_MQTT_API_REQ_TOPIC, topic_len) != 0) return;

    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) {
        ESP_LOGE(TAG, "Invalid JSON Request");
        return;
    }

    cJSON *response_root = cJSON_CreateObject();
    
    cJSON *op_item = cJSON_GetObjectItem(root, "op");
    if (!cJSON_IsString(op_item)) {
        cJSON_AddStringToObject(response_root, "status", "error");
        cJSON_AddStringToObject(response_root, "message", "Missing 'op' field (get/set)");
    } else {
        const char *op = op_item->valuestring;
        cJSON_AddStringToObject(response_root, "op", "resp");

        if (strcmp(op, "get") == 0) {
            cJSON *resource_item = cJSON_GetObjectItem(root, "resource");
            if (cJSON_IsString(resource_item) && (resource_item->valuestring != NULL)) {
                const char *resource = resource_item->valuestring;
                ESP_LOGI(TAG, "API Request for resource: %s", resource);
                cJSON_AddStringToObject(response_root, "resource", resource);

                if (strcmp(resource, "battery") == 0) handle_resource_battery(response_root);
                else if (strcmp(resource, "encoder") == 0) handle_resource_encoder(response_root);
                else if (strcmp(resource, "uptime") == 0) handle_resource_uptime(response_root);
                else if (strcmp(resource, "status") == 0) handle_resource_status(response_root);
                else if (strcmp(resource, "all") == 0) {
                    handle_resource_battery(response_root);
                    handle_resource_encoder(response_root);
                    handle_resource_uptime(response_root);
                    handle_resource_status(response_root);
                } else {
                    cJSON_AddStringToObject(response_root, "error", "Unknown Resource");
                }
            } else {
                cJSON_AddStringToObject(response_root, "error", "Missing Resource");
            }
        } 
        else if (strcmp(op, "set") == 0) {
            cJSON *action_item = cJSON_GetObjectItem(root, "action");
            if (cJSON_IsString(action_item) && (action_item->valuestring != NULL)) {
                const char *action = action_item->valuestring;
                ESP_LOGI(TAG, "API Command for action: %s", action);
                cJSON_AddStringToObject(response_root, "action", action);
                
                if (strcmp(action, "play_sound") == 0) handle_action_play_sound(root, response_root);
                else if (strcmp(action, "set_mode") == 0) handle_action_set_mode(root, response_root);
                else if (strcmp(action, "set_cal_mask") == 0) handle_action_set_cal_mask(root, response_root);
                else if (strcmp(action, "sync_time") == 0) handle_action_sync_time(root, response_root);
                else {
                    cJSON_AddStringToObject(response_root, "status", "error");
                    cJSON_AddStringToObject(response_root, "message", "Unknown Action");
                }
            } else {
                cJSON_AddStringToObject(response_root, "error", "Missing Action");
            }
        } else {
            cJSON_AddStringToObject(response_root, "status", "error");
            cJSON_AddStringToObject(response_root, "message", "Invalid 'op' value");
        }
    }

    // Stringify and publish response if anything was added
    if (response_root->child != NULL) { // only publish if not empty
        char *json_str = cJSON_PrintUnformatted(response_root);
        if (json_str != NULL) {
            mqtt_custom_client_publish(CONFIG_MQTT_API_RESP_TOPIC, json_str, 0, 0, 0); // QoS0 is fine for polls
            free(json_str);
        }
    }

    // Clean memory
    cJSON_Delete(response_root);
    cJSON_Delete(root);
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t mqtt_api_responder_init(void) {
    return mqtt_custom_client_register_topic_callback(CONFIG_MQTT_API_REQ_TOPIC, api_mqtt_callback);
}

esp_err_t mqtt_api_responder_subscribe(void) {
    if (!mqtt_custom_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    int res = mqtt_custom_client_subscribe(CONFIG_MQTT_API_REQ_TOPIC, 0);
    return (res < 0) ? ESP_FAIL : ESP_OK;
}
