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

// =============================================================================
// Application Entry Point
// =============================================================================

/**
 * Main entrance to the Lurloc firmware.
 * Responsibilities:
 * 1. Initialize low-level system services.
 * 2. Launch concurrent RTOS tasks pinned to specific cores.
 */
void app_main(void)
{
    // Initialize shared services, network and hardware drivers
    system_init();

    // Start communication task on CPU 1 (handles MQTT, logs, etc)
    task_comms_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Comms Task");

    vTaskDelay(pdMS_TO_TICKS(100));

    // Start real-time control loop on CPU 0 (PID, Sensors, Odometry)
    task_rtcontrol_cpu0_start();
    printf("[CPU%d] %-40s [ OK ]\n", 0, "Started RT Control");

    // Start monitoring and safety task on CPU 1
    task_monitor_lowpower_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Monitor Task");
}
