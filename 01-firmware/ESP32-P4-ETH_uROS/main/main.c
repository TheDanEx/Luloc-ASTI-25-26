#include "esp_log.h"
#include "esp_err.h"
#include "uros_network_interfaces.h"
#include "driver/uart.h"

#include "audio_player.h"
#include "shared_memory.h"
#include "task_rtcontrol_cpu0.h"
#include "uros_manager.h"
#include "system_init.h"
#include "task_comms_cpu1.h"
#include "task_monitor_lowpower_cpu1.h"
#include "performance_monitor.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    system_init();

    /* Bump console UART to 2M baud for fast debug output */
    uart_set_baudrate(0, 2000000);
    ESP_LOGI(TAG, "Console baud set to 2M");

    /* Init performance monitor for task timing */
    perf_mon_init();

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(uros_manager_start());
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started microROS manager Task");

    task_comms_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Comms Task");

    vTaskDelay(pdMS_TO_TICKS(100));

    task_rtcontrol_cpu0_start();
    printf("[CPU%d] %-40s [ OK ]\n", 0, "Started RT Control");

    task_monitor_lowpower_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Monitor Task");

    ESP_LOGI(TAG, "System started successfully");
}
