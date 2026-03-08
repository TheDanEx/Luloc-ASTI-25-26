#include "pid_tuner.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "mqtt_custom_client.h"
#include "shared_memory.h"

static const char *TAG = "PID_TUNER";
static const char *NVS_NAMESPACE = "robot_pids";

#ifndef CONFIG_PID_TUNER_MQTT_TOPIC
#define CONFIG_PID_TUNER_MQTT_TOPIC "robot/config/pid_motors"
#endif

// =============================================================================
// NVS Loading / Saving
// =============================================================================

esp_err_t pid_tuner_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t pid_tuner_load_motor_pid(float *kp, float *ki, float *kd) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "NVS Open Failed (first boot?), using Kconfig defaults");
        return err;
    }

    uint32_t raw_kp = 0, raw_ki = 0, raw_kd = 0;
    
    // We store floats directly natively casted as uint32_t to bypass NVS float lack natively easily
    if (nvs_get_u32(my_handle, "m_kp", &raw_kp) == ESP_OK) *kp = *((float*)&raw_kp);
    if (nvs_get_u32(my_handle, "m_ki", &raw_ki) == ESP_OK) *ki = *((float*)&raw_ki);
    if (nvs_get_u32(my_handle, "m_kd", &raw_kd) == ESP_OK) *kd = *((float*)&raw_kd);

    nvs_close(my_handle);
    ESP_LOGI(TAG, "Loaded PIDs from NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f", *kp, *ki, *kd);
    return ESP_OK;
}

esp_err_t pid_tuner_save_motor_pid(float kp, float ki, float kd) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for write!");
        return err;
    }

    uint32_t raw_kp = *((uint32_t*)&kp);
    uint32_t raw_ki = *((uint32_t*)&ki);
    uint32_t raw_kd = *((uint32_t*)&kd);

    err = nvs_set_u32(my_handle, "m_kp", raw_kp);
    err |= nvs_set_u32(my_handle, "m_ki", raw_ki);
    err |= nvs_set_u32(my_handle, "m_kd", raw_kd);
    
    err |= nvs_commit(my_handle);
    nvs_close(my_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved PIDs to NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki, kd);
    } else {
        ESP_LOGE(TAG, "Failed to commit PIDs to NVS");
    }
    
    return err;
}


// =============================================================================
// MQTT Subscriptions
// =============================================================================

static void pid_mqtt_callback(const char *topic, int topic_len, const char *data, int data_len) {
    // Safety check size
    if (topic_len <= 0 || data_len <= 0 || data_len > 512) return;

    // We only care about our specific topic
    if (strncmp(topic, CONFIG_PID_TUNER_MQTT_TOPIC, topic_len) != 0) return;

    // Parse JSON
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) {
        ESP_LOGE(TAG, "Invalid JSON received for PID tuning");
        return;
    }

    float current_kp = 0.0f, current_ki = 0.0f, current_kd = 0.0f;
    pid_tuner_load_motor_pid(&current_kp, &current_ki, &current_kd);

    bool updated = false;

    cJSON *kp_item = cJSON_GetObjectItem(root, "kp");
    if (cJSON_IsNumber(kp_item)) {
        current_kp = kp_item->valuedouble;
        updated = true;
    }

    cJSON *ki_item = cJSON_GetObjectItem(root, "ki");
    if (cJSON_IsNumber(ki_item)) {
        current_ki = ki_item->valuedouble;
        updated = true;
    }

    cJSON *kd_item = cJSON_GetObjectItem(root, "kd");
    if (cJSON_IsNumber(kd_item)) {
        current_kd = kd_item->valuedouble;
        updated = true;
    }
    
    cJSON_Delete(root);

    if (updated) {
        ESP_LOGW(TAG, "Live PID Update Received! Kp:%.3f, Ki:%.3f, Kd:%.3f", current_kp, current_ki, current_kd);
        
        // 1. Save to Flash Storage immediately
        pid_tuner_save_motor_pid(current_kp, current_ki, current_kd);
        
        // 2. Inject to Shared Memory so CPU0 (RT Control Task) applies it on the next loop tick
        shared_memory_t* shm = shared_memory_get();
        if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            shm->live_pid.kp = current_kp;
            shm->live_pid.ki = current_ki;
            shm->live_pid.kd = current_kd;
            shm->live_pid.updated_flag = true;
            xSemaphoreGive(shm->mutex);
        } else {
            ESP_LOGE(TAG, "Timeout taking shared memory mutex to inject PIDs");
        }
    }
}

esp_err_t pid_tuner_register_callback(void) {
    return mqtt_custom_client_register_topic_callback(CONFIG_PID_TUNER_MQTT_TOPIC, pid_mqtt_callback);
}

esp_err_t pid_tuner_subscribe(void) {
    if (!mqtt_custom_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    int res = mqtt_custom_client_subscribe(CONFIG_PID_TUNER_MQTT_TOPIC, 0);
    return (res < 0) ? ESP_FAIL : ESP_OK;
}
