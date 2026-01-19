#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_monitor.h"

static void task_monitor(void *arg)
{
    while(1) {
        // TODO: monitorización básica
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_monitor_start(void)
{
    xTaskCreatePinnedToCore(task_monitor, "task_monitor", 2048, NULL, 1, NULL, 1);
}
