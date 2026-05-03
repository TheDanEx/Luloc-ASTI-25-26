#include "system_init.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "state_machine.h"
#include "audio_player.h"
#include "shared_memory.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"


// =============================================================================
// System Initialization Flow
// =============================================================================

/**
 * Configure the minimal environment needed for the robot to operate.
 * This includes flash memory, the network stack, and core hardware components.
 */
void system_init(void)
{
    esp_log_level_set("i2c", ESP_LOG_ERROR);
    esp_log_level_set("ES8311", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);
    esp_log_level_set("aud_play", ESP_LOG_WARN);
    esp_log_level_set("shrd_mem", ESP_LOG_WARN);
    esp_log_level_set("esp_eth.netif.netif_glue", ESP_LOG_WARN);
    esp_log_level_set("main_task", ESP_LOG_WARN);
    esp_log_level_set("perf_mon", ESP_LOG_WARN);
    
    esp_log_level_set("comms_c1", ESP_LOG_INFO);
    esp_log_level_set("INA226", ESP_LOG_INFO);
    
    ESP_ERROR_CHECK(nvs_flash_init());
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started NVS Flash");
    
    ESP_ERROR_CHECK(esp_netif_init());
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Network Interface");

    if (audio_player_init() == ESP_OK) {
        printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Audio Player");
        audio_player_play(STARTUP);
    } else {
        printf("[CPU%d] %-40s [ ERROR ]\n", xPortGetCoreID(), "Started Audio Player");
    }
    
    shared_memory_init();
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started Shared Memory");
    state_machine_init();
    printf("[CPU%d] %-40s [ OK ]\n", xPortGetCoreID(), "Started State Machine");
}
