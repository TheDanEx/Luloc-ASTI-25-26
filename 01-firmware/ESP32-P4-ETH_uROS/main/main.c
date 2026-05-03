#include "esp_log.h"
#include "esp_err.h"
#include "uros_network_interfaces.h"

#include "audio_player.h"
#include "shared_memory.h"
#include "task_rtcontrol_cpu0.h"
#include "uros_manager.h"
#include "system_init.h"
#include "task_comms_cpu1.h"
#include "task_monitor_lowpower_cpu1.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    system_init();

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(uros_manager_start());

     // Start communication task on CPU 1 (handles MQTT, logs, etc)
    task_comms_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Comms Task");

    vTaskDelay(pdMS_TO_TICKS(100));

    task_rtcontrol_cpu0_start();
    printf("[CPU%d] %-40s [ OK ]\n", 0, "Started RT Control");

    // Start monitoring and safety task on CPU 1
    task_monitor_lowpower_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Monitor Task");

    ESP_LOGI(TAG, "System started successfully");
}
