#include "esp_log.h"
#include "esp_err.h"
#include "uros_network_interfaces.h"

#include "shared_memory.h"
#include "uros_manager.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    shared_memory_init();

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(uros_manager_start());

    ESP_LOGI(TAG, "System started successfully");
}