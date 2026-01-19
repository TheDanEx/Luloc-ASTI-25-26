#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_comms.h"

static void task_comms(void *arg)
{
    while(1) {
        // TODO: enviar datos por MQTT / Ethernet
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_comms_start(void)
{
    xTaskCreatePinnedToCore(task_comms, "task_comms", 4096, NULL, 5, NULL, 1);
}
