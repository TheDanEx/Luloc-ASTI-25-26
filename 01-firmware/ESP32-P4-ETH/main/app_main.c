/* ESP32-P4 Robot Controller with Ethernet and MQTT

   Combines Ethernet connectivity with MQTT for remote control
*/

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "system_init.h"
#include "task_rtcontrol_cpu0.h"
#include "task_comms_cpu1.h"
#include "task_monitor_lowpower_cpu1.h"



void app_main(void)
{
    // ========== SYSTEM INITIALIZATION ==========
    // Handles NVS, Netif, Ethernet, Audio, Logging config, and Shared Memory
    system_init();
    
    // ========== BASIC INFO ==========
    // ========== START TASKS ==========
    
    // 1. Communications Task (Core 1)
    task_comms_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Comms Task");
    
    // Allow brief stabilization
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 2. Real-Time Control Task (Core 0)
    task_rtcontrol_cpu0_start();
    printf("[CPU%d] %-40s [ OK ]\n", 0, "Started RT Control");

    // 3. Low-Power Monitor Task (Core 1)
    task_monitor_lowpower_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Monitor Task");
}
