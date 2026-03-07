/*
 * Task Monitor Low Power CPU1
 * Low-priority task for secondary monitoring only.
 * SPDX-License-Identifier: MIT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "test_sensor.h"
#include "task_monitor_lowpower_cpu1.h"

static const char *TAG = "mon_cpu1";

static void task_monitor_lowpower_cpu1(void *arg)
{
    (void)arg;

    if (test_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize test sensor");
        vTaskDelete(NULL);
    }

    char uptime_str[64];

    vTaskDelay(pdMS_TO_TICKS(10000));

    while (1) {
        test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
        ESP_LOGI(TAG, "Uptime: %s", uptime_str);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void task_monitor_lowpower_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_monitor_lowpower_cpu1, "monitor_cpu1", 4096, NULL, 1, NULL, 1);
}
