#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mqtt_custom_client.h"
#include "state_machine.h"
#include "shared_memory.h"
#include "task_comms_cpu1.h"
#include <string.h>
#include "audio_player.h"
#include "esp_timer.h"
#include "test_sensor.h"
#include "performance_monitor.h"
#include "ethernet.h"

static const char *TAG = "comms_c1";

// Inter-core communication queue for commands from CPU0
// Queue size: 32 commands, each command up to 256 bytes
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;

/**
 * Callback for robot/cmd topic - receives mode change commands
 */
static void mqtt_cmd_callback(const char *topic, int topic_len, const char *data, int data_len)
{
    // Parse command: "CMD_MODE:<ID>", "CMD_PLAY_SOUND:<ID>:<VOL>", etc.
    char cmd_str[64] = {0};
    strncpy(cmd_str, data, (data_len < sizeof(cmd_str) - 1) ? data_len : sizeof(cmd_str) - 1);
    cmd_str[(data_len < sizeof(cmd_str) - 1) ? data_len : sizeof(cmd_str) - 1] = '\0';
    
    ESP_LOGI(TAG, "Received command: '%s'", cmd_str);
    
    int mode_id = -1;

    if (strncmp(cmd_str, "CMD_MODE:", 9) == 0) {
        if (sscanf(cmd_str, "CMD_MODE:%d", &mode_id) == 1) {
            if (mode_id >= MODE_NONE && mode_id < MODE_COUNT) {
                // Request mode change
                if (state_machine_request_mode((robot_mode_t)mode_id)) {
                    ESP_LOGI(TAG, "Mode changed to: %d", mode_id);
                    // Send JSON Event
                    char event_json[128];
                    snprintf(event_json, sizeof(event_json), "{\"event\":\"MODE_CHANGE\",\"mode\":%d,\"mode_str\":\"%s\"}", 
                             mode_id, get_mode_name((robot_mode_t)mode_id));
                    mqtt_custom_client_publish("robot/events", event_json, 0, 1, 0);
                } else {
                    ESP_LOGW(TAG, "Mode change rejected: %d", mode_id);
                    mqtt_custom_client_publish("robot/events", "{\"error\":\"MODE_CHANGE_REJECTED\"}", 0, 1, 0);
                }
            }
        }
        return;
    } else if (strncmp(cmd_str, "MODE_AUTONOMOUS", 15) == 0) {
        state_machine_request_mode(MODE_AUTONOMOUS_PATH);
    } else if (strncmp(cmd_str, "MODE_REMOTE_DRIVE", 17) == 0) {
        state_machine_request_mode(MODE_REMOTE_DRIVE);
    } else if (strncmp(cmd_str, "MODE_TELEMETRY_STREAM", 21) == 0) {
        state_machine_request_mode(MODE_TELEMETRY_STREAM);
    } else if (strncmp(cmd_str, "CMD_PLAY_SOUND", 14) == 0) {
        // Format: CMD_PLAY_SOUND:ID:[VOLUME]
        int sound_id = 0;
        int volume = -1;
        // Try parsing with volume first
        if (sscanf(cmd_str, "CMD_PLAY_SOUND:%d:%d", &sound_id, &volume) == 2) {
             #include "audio_player.h" 
             if (sound_id >= 0 && sound_id < SOUND_MAX) {
                 audio_player_play_vol((audio_sound_t)sound_id, (uint8_t)volume);
                 ESP_LOGI(TAG, "Playing sound %d at vol %d", sound_id, volume);
             }
        } else if (sscanf(cmd_str, "CMD_PLAY_SOUND:%d", &sound_id) == 1) {
             if (sound_id >= 0 && sound_id < SOUND_MAX) {
                 audio_player_play((audio_sound_t)sound_id);
                 ESP_LOGI(TAG, "Playing sound %d", sound_id);
             }
        }
        return;
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_str);
        return;
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
    // Initialize command queue for inter-core communication
    task_comms_cpu1_init_queue();
    
    // Wait for Ethernet connection (IP obtained)
    // Wait for Ethernet connection (IP obtained)
    // ESP_LOGI(TAG, "Waiting for Ethernet connection...");
    while (!ethernet_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Ethernet connected - Starting MQTT...");

    // All MQTT tasks will inherit this core pinning
    if (mqtt_custom_client_init() != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: Failed to initialize MQTT client - communications disabled");
        mqtt_initialized = false;
        vTaskDelete(NULL);
    }
    
    mqtt_initialized = true;
    ESP_LOGD(TAG, "MQTT client initialized - running in CPU%d context", xPortGetCoreID());
    
    // Wait a bit for MQTT to establish connection
    // We can rely on auto-reconnect, but a small delay helps
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Subscribe to robot/cmd
    mqtt_custom_client_subscribe("robot/cmd", 1);
    
    // Wait for subscription to be processed
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Register callback for subscribed topics
    esp_err_t cb_err = mqtt_custom_client_register_topic_callback("robot/cmd", mqtt_cmd_callback);
    if (cb_err != ESP_OK) {
        ESP_LOGD(TAG, "✓ Callback registered successfully");
    }
    
    // Initialize Performance Monitor
    perf_mon_init();
    
    // Telemetry rate control
    const TickType_t telemetry_interval = pdMS_TO_TICKS(5000);
    TickType_t last_telemetry_time = 0;

    while(1) {
        // High-speed command polling (keep responsive)
        uint8_t command_buffer[256];
        if (xQueueReceive(command_queue, command_buffer, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Command received from CPU0: %s", (char*)command_buffer);
            // Process command here if needed
        }
        
        TickType_t now = xTaskGetTickCount();
        
        // Publish telemetry every 5 seconds
        if ((now - last_telemetry_time) >= telemetry_interval) {
            test_sensor_data_t sensor_data;
            if (test_sensor_read(&sensor_data) == ESP_OK) {
                char json_payload[64];
                snprintf(json_payload, sizeof(json_payload), "{\"uptime_seconds\":%lu.%03lu}", 
                         sensor_data.uptime_sec, sensor_data.uptime_ms % 1000);
                if (mqtt_custom_client_is_connected()) {
                    mqtt_custom_client_publish("robot/telemetry", json_payload, 0, 0, 0);
                }
            }
            
            // Update performance stats (calculates usage %)
            if (perf_mon_update() == ESP_OK) {
                // Print Performance Report to Console
                perf_mon_print_report();

                // Publish Performance Stats (JSON Usage %)
                char *perf_json = malloc(2048);
                if (perf_json) {
                    if (perf_mon_get_report_json(perf_json, 2048) == ESP_OK) {
                       if (mqtt_custom_client_is_connected()) {
                           mqtt_custom_client_publish("robot/performance", perf_json, 0, 0, 0);
                       }
                    }
                    free(perf_json);
                }
            }
            
            last_telemetry_time = now;
        }

        // 10ms polling interval for responsiveness
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_comms_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 8192, NULL, 5, NULL, 1);
}
