#include "esp_log.h"
#include "esp_err.h"
#include "uros_network_interfaces.h"

#include "audio_player.h"
#include "shared_memory.h"
#include "state_machine.h"
#include "task_rtcontrol_cpu0.h"
#include "uros_manager.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    shared_memory_init();
    state_machine_init();

    esp_err_t audio_err = audio_player_init();
    if (audio_err != ESP_OK) {
        ESP_LOGW(TAG, "Audio player unavailable: %s", esp_err_to_name(audio_err));
    }

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_ERROR_CHECK(uros_manager_start());
    task_rtcontrol_cpu0_start();

    ESP_LOGI(TAG, "System started successfully");
}
