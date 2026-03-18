#include "system_init.h"

#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "ethernet.h"
#include "audio_player.h"
#include "shared_memory.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void system_init(void)
{
    // 1. Log Level Configuration - Suppress sub-modules for clean boot
    // Default level is set to WARN in sdkconfig to hide early boot logs
    // esp_log_level_set("*", ESP_LOG_INFO); // Don't restore INFO globally yet
    esp_log_level_set("i2c", ESP_LOG_ERROR);
    esp_log_level_set("ES8311", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);
    
    // Suppress component INFO logs to handle printing here
    esp_log_level_set("ethernet", ESP_LOG_INFO);
    esp_log_level_set("aud_play", ESP_LOG_WARN);
    esp_log_level_set("shrd_mem", ESP_LOG_WARN);
    esp_log_level_set("esp_eth.netif.netif_glue", ESP_LOG_WARN); // Suppress glue attach logs
    esp_log_level_set("main_task", ESP_LOG_WARN); // Suppress 'Calling app_main' etc
    esp_log_level_set("perf_mon", ESP_LOG_WARN);  // Suppress perf mon init trace
    esp_log_level_set("mqtt_custom_client", ESP_LOG_WARN); // Suppress MQTT info logs (keep warnings)
    // Enable comms task logs for debugging (show encoder speed on serial)
    esp_log_level_set("comms_c1", ESP_LOG_INFO);
    esp_log_level_set("curv_ff", ESP_LOG_INFO);
    
    // 2. Core Subsystems
    ESP_ERROR_CHECK(nvs_flash_init());
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started NVS Flash");
    
    ESP_ERROR_CHECK(esp_netif_init());
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Network Interface");
    
    // 3. Hardware Initializations
    ESP_ERROR_CHECK(ethernet_init());
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Ethernet Driver");
    
    if (audio_player_init() == ESP_OK) {
        printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Audio Player");
        audio_player_play(STARTUP);
    } else {
        printf("[CPU%d] %-40s [ ERROR ]\n", xPortGetCoreID(), "Started Audio Player");
    }
    
    shared_memory_init();
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Shared Memory");
}
