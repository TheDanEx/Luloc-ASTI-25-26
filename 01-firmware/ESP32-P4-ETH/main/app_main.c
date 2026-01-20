#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_init.h"
#include "tasks/task_control.h"
#include "tasks/task_comms.h"
#include "tasks/task_monitor.h"
#include "ethernet_init.h"
#include "logger.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "app_main";

static void task_print(void *arg)
{
    while (1) {
        logger_send(LOG_INFO, "Prueba de vida");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    system_init();
    logger_init();
    printf("UART por defecto, 115200 baudios\n");

    xTaskCreate(
        task_print,        // función
        "task_print",      // nombre
        2048,              // stack (suficiente para printf)
        NULL,              // argumento
        5,                 // prioridad
        NULL               // handle
    );

    // Lanzamos las tasks ancladas a cores
    task_control_start();   // Core 0
    task_comms_start();     // Core 1
    task_monitor_start();   // opcional

#ifdef CONFIG_ROBOT_USE_ETHERNET
    // Initialize Ethernet stack
    if (ethernet_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Ethernet");
    } else {
        ESP_LOGI(TAG, "Ethernet initialized successfully");
    }
#else
    ESP_LOGI(TAG, "Ethernet disabled in configuration");
#endif
}
