#include "mqtt_watchdog.h"
#include "mqtt_custom_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mqtt_watchdog";

typedef struct {
    mqtt_health_status_t status;
    TickType_t last_connected_time;
    TickType_t disconnect_start_time;
    uint32_t disconnect_threshold_ms;  // Default 5000ms
    bool enabled;
    uint32_t reconnect_attempts;
} mqtt_watchdog_ctx_t;

static mqtt_watchdog_ctx_t g_watchdog = {
    .status = MQTT_HEALTH_DISCONNECTED,
    .last_connected_time = 0,
    .disconnect_start_time = 0,
    .disconnect_threshold_ms = 5000,
    .enabled = true,
    .reconnect_attempts = 0
};

void mqtt_watchdog_init(void)
{
    g_watchdog.status = MQTT_HEALTH_DISCONNECTED;
    g_watchdog.last_connected_time = xTaskGetTickCount();
    g_watchdog.disconnect_start_time = xTaskGetTickCount();
    g_watchdog.reconnect_attempts = 0;
    g_watchdog.enabled = true;
    
    ESP_LOGI(TAG, "MQTT Watchdog initialized - threshold: %"PRIu32"ms", 
             g_watchdog.disconnect_threshold_ms);
}

mqtt_health_status_t mqtt_watchdog_check(void)
{
    if (!g_watchdog.enabled) {
        return g_watchdog.status;
    }

    bool currently_connected = mqtt_custom_client_is_connected();
    TickType_t now = xTaskGetTickCount();
    
    // State machine
    switch (g_watchdog.status) {
    case MQTT_HEALTH_CONNECTED:
        if (currently_connected) {
            // Still connected, update timestamp
            g_watchdog.last_connected_time = now;
        } else {
            // Connection lost
            g_watchdog.status = MQTT_HEALTH_DISCONNECTED;
            g_watchdog.disconnect_start_time = now;
            ESP_LOGW(TAG, "Connection lost - starting watchdog");
        }
        break;
        
    case MQTT_HEALTH_DISCONNECTED:
        if (currently_connected) {
            // Recovered!
            g_watchdog.status = MQTT_HEALTH_CONNECTED;
            g_watchdog.last_connected_time = now;
            g_watchdog.reconnect_attempts = 0;
            ESP_LOGI(TAG, "Connection recovered after %"PRIu32"ms",
                     (uint32_t)((now - g_watchdog.disconnect_start_time) / portTICK_PERIOD_MS));
        } else {
            // Still disconnected - check threshold
            uint32_t disconnected_ms = (uint32_t)((now - g_watchdog.disconnect_start_time) / portTICK_PERIOD_MS);
            
            if (disconnected_ms > g_watchdog.disconnect_threshold_ms) {
                // Critical: disconnected for too long
                g_watchdog.status = MQTT_HEALTH_FAILED;
                ESP_LOGE(TAG, "CRITICAL: Connection lost for %"PRIu32"ms - SAFETY STOP activated",
                         disconnected_ms);
            } else if (disconnected_ms > 1000) {
                // Log periodic warnings
                if ((disconnected_ms % 2000) == 0) {
                    ESP_LOGW(TAG, "Disconnected for %"PRIu32"ms (threshold: %"PRIu32"ms)",
                             disconnected_ms, g_watchdog.disconnect_threshold_ms);
                }
            }
        }
        break;
        
    case MQTT_HEALTH_FAILED:
        // In failed state, try periodic reconnection
        if (currently_connected) {
            g_watchdog.status = MQTT_HEALTH_CONNECTED;
            g_watchdog.last_connected_time = now;
            ESP_LOGI(TAG, "Recovered from FAILED state after manual intervention");
        }
        break;
    }
    
    return g_watchdog.status;
}

bool mqtt_watchdog_is_connected(void)
{
    return g_watchdog.status == MQTT_HEALTH_CONNECTED;
}

uint32_t mqtt_watchdog_get_disconnected_time_ms(void)
{
    if (g_watchdog.status == MQTT_HEALTH_CONNECTED) {
        return 0;
    }
    
    TickType_t now = xTaskGetTickCount();
    return (uint32_t)((now - g_watchdog.disconnect_start_time) / portTICK_PERIOD_MS);
}

bool mqtt_watchdog_reconnect(void)
{
    ESP_LOGI(TAG, "Attempting manual reconnection (attempt #%"PRIu32")",
             ++g_watchdog.reconnect_attempts);
    
    // This would trigger MQTT client reconnect
    // Implementation depends on ESP-IDF MQTT client API
    // For now, just trigger watchdog check
    mqtt_watchdog_check();
    
    return g_watchdog.status == MQTT_HEALTH_CONNECTED;
}

void mqtt_watchdog_set_enabled(bool enabled)
{
    g_watchdog.enabled = enabled;
    if (enabled) {
        ESP_LOGI(TAG, "Watchdog enabled");
    } else {
        ESP_LOGW(TAG, "Watchdog disabled");
    }
}
