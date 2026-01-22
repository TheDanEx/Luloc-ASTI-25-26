#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mqtt_custom_client.h"
#include "task_comms_cpu1.h"

static const char *TAG = "task_comms_cpu1";

// Inter-core communication queue for commands from CPU0
// Queue size: 32 commands, each command up to 256 bytes
static QueueHandle_t command_queue = NULL;

void task_comms_cpu1_init_queue(void)
{
    if (command_queue == NULL) {
        command_queue = xQueueCreate(32, 256);
        if (command_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create command queue");
        }
    }
}

QueueHandle_t task_comms_cpu1_get_queue(void)
{
    return command_queue;
}

static void task_comms_cpu1(void *arg)
{
    ESP_LOGI(TAG, "==== COMMUNICATIONS TASK STARTED ON CORE %d ====", xPortGetCoreID());
    ESP_LOGI(TAG, "Configured for HIGH-SPEED telemetry (50Hz / 20ms)");
    
    // Initialize command queue for inter-core communication
    task_comms_cpu1_init_queue();
    
    // All MQTT tasks will inherit this core pinning
    if (mqtt_custom_client_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        vTaskDelete(NULL);
    }
    
    ESP_LOGI(TAG, "MQTT client initialized - running in CPU%d context", xPortGetCoreID());
    
    uint32_t msg_count = 0;
    TickType_t last_log_time = xTaskGetTickCount();
    
    while(1) {
        // High-speed telemetry loop: 50 Hz (20ms interval)
        // Future: Replace with actual telemetry data acquisition and JSON packing
        
        // Check for incoming commands from CPU0 (non-blocking)
        uint8_t command_buffer[256];
        if (xQueueReceive(command_queue, command_buffer, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Command received from CPU0: %s", (char*)command_buffer);
            // Process command here
        }
        
        msg_count++;
        
        // Log statistics every 5 seconds
        if ((xTaskGetTickCount() - last_log_time) > pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "Telemetry stats - Messages: %"PRIu32", Rate: %.1f Hz, Est. throughput: %.1f KB/s",
                     msg_count, (msg_count * 1000.0f) / 5000.0f, 
                     (msg_count * 1024.0f) / 5000.0f);  // Assuming ~1KB per message
            msg_count = 0;
            last_log_time = xTaskGetTickCount();
        }
        
        // 20ms interval = 50 Hz telemetry rate
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void task_comms_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 8192, NULL, 5, NULL, 1);
}
