#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_control.h"

static void task_control(void *arg)
{
    while(1) {
        // TODO: código de control
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_control_start(void)
{
    xTaskCreatePinnedToCore(task_control, "task_control", 4096, NULL, 10, NULL, 0);
}
