#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_init.h"
#include "tasks/task_control.h"
#include "tasks/task_comms.h"
#include "tasks/task_monitor.h"
#include "ethernet.h"
#include "logger.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN_ORCHESTRATOR";

static void task_print(void *arg)
{
    while (1) {
        logger_send(LOG_INFO, "Prueba de vida");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void task_navigator(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando Sistema de Navegación en Core 1...");

    // 1. Inicializar Ethernet (LwIP + Driver + PHY)
    // Esto configura la IP estática y levanta la interfaz física
    ethernet_init_with_static_ip();

    // 2. Aquí iría tu bucle de lógica de comunicaciones (MQTT)
    while (1) {
        // Simulación de actividad de red (ej. revisar mailboxes o colas)
        // En el futuro: mqtt_client_publish(...)
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void app_main(void)
{
    system_init();
    logger_init();
    printf("UART por defecto, 115200 baudios\n");
    //ethernet_init();
    xTaskCreatePinnedToCore(task_navigator, "NAVIGATOR", 4096, NULL, 5, NULL, 1);

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
}