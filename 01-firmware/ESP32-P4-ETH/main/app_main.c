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
    system_init();

    task_comms_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Comms Task");

    vTaskDelay(pdMS_TO_TICKS(100));

    task_rtcontrol_cpu0_start();
    printf("[CPU%d] %-40s [ OK ]\n", 0, "Started RT Control");

    task_monitor_lowpower_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Monitor Task");
}
