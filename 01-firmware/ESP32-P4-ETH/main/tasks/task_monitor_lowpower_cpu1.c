#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "test_sensor.h"
#include "mqtt_custom_client.h"
#include "task_monitor_lowpower_cpu1.h"

static const char *TAG = "monitor_cpu1";

static void task_monitor_lowpower_cpu1(void *arg)
{
    ESP_LOGI(TAG, "Low-power monitoring task started on core %d (low priority)", xPortGetCoreID());
    
    // Initialize test sensor
    if (test_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize test sensor");
        vTaskDelete(NULL);
    }

    char uptime_str[64];
    
    while(1) {
        // Get uptime from test sensor
        test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
        
        // Log to serial
        ESP_LOGI(TAG, "System uptime: %s", uptime_str);
        
        // Publish to MQTT every 5 seconds
        int msg_id = mqtt_custom_client_publish("robot/uptime", uptime_str, 0, 1, 0);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "Failed to publish uptime to MQTT");
        } else {
            ESP_LOGD(TAG, "Published uptime to MQTT (msg_id=%d)", msg_id);
        }
        
        // Sleep for 5 seconds (low priority, low frequency)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void task_monitor_lowpower_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_monitor_lowpower_cpu1, "monitor_cpu1", 2048, NULL, 1, NULL, 1);
}
