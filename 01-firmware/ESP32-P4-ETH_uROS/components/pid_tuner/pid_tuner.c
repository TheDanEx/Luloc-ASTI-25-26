#include "pid_tuner.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "shared_memory.h"

// =============================================================================
// Definitions & Local State
// =============================================================================

static const char *TAG = "PID_TUNER";
static const char *NVS_NAMESPACE = "robot_pids";

#ifndef CONFIG_PID_TUNER_MQTT_TOPIC
#define CONFIG_PID_TUNER_MQTT_TOPIC "robot/config/motors"
#endif

// =============================================================================
// Public API: Lifecycle & Storage
// =============================================================================

/**
 * Initialize NVS flash for persistent PID storage
 */

esp_err_t pid_tuner_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t pid_tuner_load_motor_pid(uint8_t index, float *kp, float *ki, float *kd) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "NVS Open Failed for index %d, using Kconfig defaults", index);
        return err;
    }

    uint32_t raw_kp = 0, raw_ki = 0, raw_kd = 0;
    char key_kp[8], key_ki[8], key_kd[8];
    snprintf(key_kp, sizeof(key_kp), "%c_kp", (index == 0) ? 'l' : 'r');
    snprintf(key_ki, sizeof(key_ki), "%c_ki", (index == 0) ? 'l' : 'r');
    snprintf(key_kd, sizeof(key_kd), "%c_kd", (index == 0) ? 'l' : 'r');
    
    if (nvs_get_u32(my_handle, key_kp, &raw_kp) == ESP_OK) *kp = *((float*)&raw_kp);
    if (nvs_get_u32(my_handle, key_ki, &raw_ki) == ESP_OK) *ki = *((float*)&raw_ki);
    if (nvs_get_u32(my_handle, key_kd, &raw_kd) == ESP_OK) *kd = *((float*)&raw_kd);

    nvs_close(my_handle);
    ESP_LOGI(TAG, "Loaded PIDs [%d] from NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f", index, *kp, *ki, *kd);
    return ESP_OK;
}

esp_err_t pid_tuner_save_motor_pid(uint8_t index, float kp, float ki, float kd) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for write!");
        return err;
    }

    uint32_t raw_kp = *((uint32_t*)&kp);
    uint32_t raw_ki = *((uint32_t*)&ki);
    uint32_t raw_kd = *((uint32_t*)&kd);

    char key_kp[8], key_ki[8], key_kd[8];
    snprintf(key_kp, sizeof(key_kp), "%c_kp", (index == 0) ? 'l' : 'r');
    snprintf(key_ki, sizeof(key_ki), "%c_ki", (index == 0) ? 'l' : 'r');
    snprintf(key_kd, sizeof(key_kd), "%c_kd", (index == 0) ? 'l' : 'r');

    err = nvs_set_u32(my_handle, key_kp, raw_kp);
    err |= nvs_set_u32(my_handle, key_ki, raw_ki);
    err |= nvs_set_u32(my_handle, key_kd, raw_kd);
    
    err |= nvs_commit(my_handle);
    nvs_close(my_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved PIDs [%d] to NVS: Kp=%.3f, Ki=%.3f, Kd=%.3f", index, kp, ki, kd);
    } else {
        ESP_LOGE(TAG, "Failed to commit PIDs to NVS");
    }
    
    return err;
}


// =============================================================================
// Public API: Configuration
// =============================================================================

/**
 * Register the PID tuning callback with the MQTT client
 */
esp_err_t pid_tuner_register_callback(void) {
    ESP_LOGW(TAG, "MQTT PID tuning disabled in teleoperation-only build");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t pid_tuner_subscribe(void) {
    ESP_LOGW(TAG, "MQTT PID tuning disabled in teleoperation-only build");
    return ESP_ERR_NOT_SUPPORTED;
}

bool pid_tuner_check_and_clear_update(uint8_t index, float *kp, float *ki, float *kd) {
    if (index >= 2) return false;
    
    shared_memory_t* shm = shared_memory_get();
    bool updated = false;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (shm->motor_pids[index].updated_flag) {
            *kp = shm->motor_pids[index].kp;
            *ki = shm->motor_pids[index].ki;
            *kd = shm->motor_pids[index].kd;
            shm->motor_pids[index].updated_flag = false;
            updated = true;
        }
        xSemaphoreGive(shm->mutex);
    }
    
    return updated;
}
