/* ESP32-P4 Robot Controller with Ethernet and MQTT

   Combines Ethernet connectivity with MQTT for remote control
*/

#include <stdio.h>
#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "system_init.h"
#include "shared_memory.h"
#include "task_rtcontrol_cpu0.h"
#include "task_comms_cpu1.h"
#include "task_monitor_lowpower_cpu1.h"

static const char *TAG = "app_main";

void app_main(void)
{
    // ========== BASIC INITIALIZATION ==========
    ESP_LOGI(TAG, "[APP] Startup - Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    esp_log_level_set("*", ESP_LOG_INFO);

    // Initialize core subsystems
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(ethernet_init());

    // ========== SYSTEM INITIALIZATION ==========
    shared_memory_init();  // Inter-core communication

    // ========== START TASKS ==========
    // Start communications task FIRST (will initialize MQTT in CPU1 context)
    task_comms_cpu1_start();
    
    // Wait up to 2 seconds for MQTT to initialize
    uint32_t wait_time = 0;
    while (!task_comms_cpu1_is_ready() && wait_time < 2000) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_time += 100;
    }
    
    // Check if communications are ready
    if (!task_comms_cpu1_is_ready()) {
        ESP_LOGE(TAG, "[APP] FATAL: Communications task failed to initialize MQTT");
        ESP_LOGE(TAG, "[APP] System halted - real-time tasks NOT started");
        ESP_LOGE(TAG, "[APP] Check network connectivity and MQTT broker configuration");
        
        // Loop indefinitely with watchdog disabled (or implement graceful shutdown)
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            ESP_LOGE(TAG, "[APP] Waiting for manual intervention...");
        }
    }
    
    // Communications OK - start real-time tasks
    ESP_LOGI(TAG, "[APP] Communications initialized successfully");
    task_rtcontrol_cpu0_start();
    task_monitor_lowpower_cpu1_start();
    ESP_LOGI(TAG, "[APP] System running on dual cores (HIGH-SPEED TELEMETRY MODE)");
}
