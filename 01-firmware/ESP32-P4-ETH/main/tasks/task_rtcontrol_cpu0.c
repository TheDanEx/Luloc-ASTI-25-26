#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_rtcontrol_cpu0.h"

static const char *TAG = "task_rtcontrol_cpu0";

static void task_rtcontrol_cpu0(void *arg)
{
    ESP_LOGI(TAG, "Real-time control task started on core %d (high priority)", xPortGetCoreID());
    
    while(1) {
        // Motor control, sensor reading, real-time algorithms
        ESP_LOGD(TAG, "Control loop running");
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
}

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 4096, NULL, 10, NULL, 0);
}
