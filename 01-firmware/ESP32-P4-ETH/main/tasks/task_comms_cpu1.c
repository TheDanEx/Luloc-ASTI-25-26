#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mqtt_custom_client.h"
#include "state_machine.h"
#include "shared_memory.h"
#include "task_comms_cpu1.h"
#include <string.h>

static const char *TAG = "task_comms_cpu1";

// Inter-core communication queue for commands from CPU0
// Queue size: 32 commands, each command up to 256 bytes
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;

/**
 * Callback for robot/cmd topic - receives mode change commands
 */
static void mqtt_cmd_callback(const char *topic, int topic_len, const char *data, int data_len)
{
    ESP_LOGI(TAG, "========== mqtt_cmd_callback INVOKED ==========");
    ESP_LOGI(TAG, "Topic: %.*s (len=%d), Data len: %d", topic_len, topic, topic_len, data_len);
    
    // Parse command: "MODE_AUTONOMOUS", "MODE_REMOTE_DRIVE", "MODE_TELEMETRY_STREAM", etc.
    char cmd_str[64] = {0};
    strncpy(cmd_str, data, (data_len < sizeof(cmd_str) - 1) ? data_len : sizeof(cmd_str) - 1);
    cmd_str[(data_len < sizeof(cmd_str) - 1) ? data_len : sizeof(cmd_str) - 1] = '\0';
    
    ESP_LOGI(TAG, "Received command: '%s'", cmd_str);
    
    robot_mode_t new_mode = MODE_NONE;
    
    if (strncmp(cmd_str, "MODE_AUTONOMOUS", 15) == 0) {
        new_mode = MODE_AUTONOMOUS_PATH;
    } else if (strncmp(cmd_str, "MODE_REMOTE_DRIVE", 17) == 0) {
        new_mode = MODE_REMOTE_DRIVE;
    } else if (strncmp(cmd_str, "MODE_TELEMETRY_STREAM", 21) == 0) {
        new_mode = MODE_TELEMETRY_STREAM;
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_str);
        return;
    }
    
    // Request mode change in state machine
    if (state_machine_request_mode(new_mode)) {
        ESP_LOGI(TAG, "Mode changed to: %d", new_mode);
        mqtt_custom_client_publish("robot/events", cmd_str, 0, 1, 0);
    } else {
        ESP_LOGW(TAG, "Mode change rejected: %d", new_mode);
        mqtt_custom_client_publish("robot/events", "ERROR_MODE_CHANGE_REJECTED", 0, 1, 0);
    }
}

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

bool task_comms_cpu1_is_ready(void)
{
    return mqtt_initialized;
}

static void task_comms_cpu1(void *arg)
{
    ESP_LOGI(TAG, "==== COMMUNICATIONS TASK STARTED ON CORE %d ====", xPortGetCoreID());
    ESP_LOGI(TAG, "Configured for HIGH-SPEED telemetry (50Hz / 20ms)");
    
    // Initialize command queue for inter-core communication
    task_comms_cpu1_init_queue();
    
    // All MQTT tasks will inherit this core pinning
    if (mqtt_custom_client_init() != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: Failed to initialize MQTT client - communications disabled");
        mqtt_initialized = false;
        vTaskDelete(NULL);
    }
    
    mqtt_initialized = true;
    ESP_LOGI(TAG, "MQTT client initialized - running in CPU%d context", xPortGetCoreID());
    
    // Wait a bit for MQTT to establish connection
    ESP_LOGI(TAG, "Waiting for MQTT connection to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Subscribe to command topic FIRST
    ESP_LOGI(TAG, "Subscribing to robot/cmd with QoS=1...");
    int sub_err = mqtt_custom_client_subscribe("robot/cmd", 1);
    ESP_LOGI(TAG, "Subscribe result: msg_id=%d", sub_err);
    
    // Wait for subscription to be processed
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // THEN register callback for subscribed topics
    ESP_LOGI(TAG, "Registering callback for robot/cmd topic...");
    esp_err_t cb_err = mqtt_custom_client_register_topic_callback("robot/cmd", mqtt_cmd_callback);
    if (cb_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callback for robot/cmd: 0x%x", cb_err);
    } else {
        ESP_LOGI(TAG, "✓ Callback registered successfully");
    }
    
    
    while(1) {
        // High-speed telemetry loop: 100 Hz (10ms interval)
        // Future: Replace with actual telemetry data acquisition and JSON packing
        
        // Check for incoming commands from CPU0 (non-blocking)
        uint8_t command_buffer[256];
        if (xQueueReceive(command_queue, command_buffer, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Command received from CPU0: %s", (char*)command_buffer);
            // Process command here
        }
        
        // 20ms interval = 100 Hz telemetry rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_comms_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 8192, NULL, 5, NULL, 1);
}
