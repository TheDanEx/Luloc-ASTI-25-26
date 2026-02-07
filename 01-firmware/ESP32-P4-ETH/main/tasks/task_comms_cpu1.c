/*
 * Task Comms CPU1 - Manages Communications, Telemetry, and Sensor Monitoring
 * Core: 1
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

// Components
#include "mqtt_custom_client.h"
#include "state_machine.h"
#include "shared_memory.h"
#include "audio_player.h"
#include "test_sensor.h"
#include "performance_monitor.h"
#include "ethernet.h"
#include "encoder_sensor.h"
#include "telemetry_manager.h"
#include "servo.h"
#include "driver/gpio.h"

#include "task_comms_cpu1.h"

// =============================================================================
// Constants & Config
// =============================================================================
static const char *TAG = "comms_c1";

#define CMD_QUEUE_SIZE          32
#define CMD_QUEUE_ITEM_SIZE     256
#define POLL_INTERVAL_MS        10

// Encoder Configuration
#define ENC_PIN_A               GPIO_NUM_18
#define ENC_PIN_B               GPIO_NUM_19
#define ENC_PPR_MOTOR           11
#define ENC_WHEEL_DIA_M         0.063f  // 6.3 cm
#define ENC_GEAR_RATIO          73.0f   // 73:1 reduction

// =============================================================================
// Static Variables
// =============================================================================
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;
static encoder_sensor_handle_t encoder_handle = NULL;
static servo_handle_t servo_handle = NULL;

// Telemetry Handles
static telemetry_handle_t tel_odometry = NULL;
static telemetry_handle_t tel_system = NULL;

// Servo animation state
static float servo_position = 0.5f;          // Current normalized position [0..1]
static float servo_direction = 0.02f;        // Speed of sweep (positive = right)
static int64_t servo_last_update_us = 0;    // Timestamp of last servo update

// =============================================================================
// Helper Function Prototypes
// =============================================================================
static void handle_mqtt_command(const char *cmd_str);
static void mqtt_cmd_callback(const char *topic, int topic_len, const char *data, int data_len);
static void collect_sensor_data(void);

// =============================================================================
// MQTT Callbacks & Command Handling
// =============================================================================

static void handle_mqtt_command(const char *cmd_str)
{
    int mode_id = -1;
    int sound_id = 0;
    int volume = -1;

    // 1. Mode Change
    if (strncmp(cmd_str, "CMD_MODE:", 9) == 0) {
        if (sscanf(cmd_str, "CMD_MODE:%d", &mode_id) == 1) {
            if (mode_id >= MODE_NONE && mode_id < MODE_COUNT) {
                if (state_machine_request_mode((robot_mode_t)mode_id)) {
                    ESP_LOGI(TAG, "Mode changed to: %d", mode_id);
                    // Events are sporadic, sending manually is fine or could make an event telemetry
                    char event_json[128];
                    snprintf(event_json, sizeof(event_json), 
                            "{\"event\":\"MODE_CHANGE\",\"mode\":%d,\"mode_str\":\"%s\"}", 
                             mode_id, get_mode_name((robot_mode_t)mode_id));
                    mqtt_custom_client_publish("robot/events", event_json, 0, 1, 0);
                } else {
                    ESP_LOGW(TAG, "Mode change rejected: %d", mode_id);
                    mqtt_custom_client_publish("robot/events", "{\"error\":\"MODE_CHANGE_REJECTED\"}", 0, 1, 0);
                }
            }
        }
    } 
    // 2. Named Modes
    else if (strncmp(cmd_str, "MODE_AUTONOMOUS", 15) == 0) {
        state_machine_request_mode(MODE_AUTONOMOUS_PATH);
    } else if (strncmp(cmd_str, "MODE_REMOTE_DRIVE", 17) == 0) {
        state_machine_request_mode(MODE_REMOTE_DRIVE);
    } else if (strncmp(cmd_str, "MODE_TELEMETRY_STREAM", 21) == 0) {
        state_machine_request_mode(MODE_TELEMETRY_STREAM);
    } 
    // 3. Sound
    else if (strncmp(cmd_str, "CMD_PLAY_SOUND", 14) == 0) {
        if (sscanf(cmd_str, "CMD_PLAY_SOUND:%d:%d", &sound_id, &volume) == 2) {
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
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_str);
    }
}

static void mqtt_cmd_callback(const char *topic, int topic_len, const char *data, int data_len)
{
    char cmd_str[64] = {0};
    int copy_len = (data_len < sizeof(cmd_str) - 1) ? data_len : (sizeof(cmd_str) - 1);
    
    strncpy(cmd_str, data, copy_len);
    cmd_str[copy_len] = '\0';
    
    ESP_LOGI(TAG, "MQTT Cmd: '%s'", cmd_str);
    handle_mqtt_command(cmd_str);
}

// =============================================================================
// Data Collection
// =============================================================================

static void collect_sensor_data(void)
{
    // 1. Encoder Data (Odometry)
    if (encoder_handle) {
        float speed = encoder_sensor_get_speed(encoder_handle);
        float distance = encoder_sensor_get_distance(encoder_handle);
        
        // Push to telemetry: field=val
        telemetry_add_float(tel_odometry, "velIZ", speed);
        telemetry_add_float(tel_odometry, "posIZ", distance);
    }

    // 2. System Data
    test_sensor_data_t sensor_data;
    if (test_sensor_read(&sensor_data) == ESP_OK) {
        telemetry_add_int(tel_system, "uptime_sec", sensor_data.uptime_sec);
        telemetry_add_int(tel_system, "uptime_ms", sensor_data.uptime_ms);
    }
    
    // 3. Performance Data
    // Perf mon generates its own complex report, but we could add summary metrics here
    // or keep using its internal json reporter if complex nested data is needed.
    // For Influx, flat fields are better.
    if (perf_mon_update() == ESP_OK) {
        // Example: telemetry_add_float(tel_system, "cpu0_usage", perf_val);
    }
}

// =============================================================================
// Public API
// =============================================================================

void task_comms_cpu1_init_queue(void)
{
    if (command_queue == NULL) {
        command_queue = xQueueCreate(CMD_QUEUE_SIZE, CMD_QUEUE_ITEM_SIZE);
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

// =============================================================================
// Main Task
// =============================================================================

static void task_comms_cpu1(void *arg)
{
    // 1. Initialization
    task_comms_cpu1_init_queue();
    
    while (!ethernet_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Ethernet connected. Initializing subsystems...");

    // MQTT
    if (mqtt_custom_client_init() != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: MQTT Init Failed");
        mqtt_initialized = false;
        vTaskDelete(NULL);
    }
    mqtt_initialized = true;

    // Encoder
    encoder_sensor_config_t enc_config = {
        .pin_a = ENC_PIN_A,
        .pin_b = ENC_PIN_B,
        .ppr = ENC_PPR_MOTOR,
        .wheel_diameter_m = ENC_WHEEL_DIA_M,
        .gear_ratio = ENC_GEAR_RATIO,
        .reverse_direction = false
    };
    encoder_handle = encoder_sensor_init(&enc_config);
    if (!encoder_handle) {
        ESP_LOGE(TAG, "Failed to init Encoder Sensor");
    }

    // Servo Motor Test (funny sweep animation)
    servo_config_t servo_cfg = {
        .gpio_num = 5,                              // GPIO 5 (not used by encoder)
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,                  // Different timer from other PWM if any
        .channel = LEDC_CHANNEL_1,
        .duty_resolution = LEDC_TIMER_13_BIT,       // 13-bit PWM
        .freq_hz = 50,                              // 50 Hz standard servo freq
        .min_pulse_us = 500,                        // 0.5ms (left)
        .max_pulse_us = 2500,                       // 2.5ms (right)
    };
    servo_handle = servo_init(&servo_cfg);
    if (!servo_handle) {
        ESP_LOGW(TAG, "Failed to init Servo (non-critical)");
    } else {
        // Start sweep from center
        servo_position = 0.5f;
        servo_move_normalized(servo_handle, servo_position);
        servo_last_update_us = esp_timer_get_time();
        ESP_LOGI(TAG, "Servo initialized and ready for fun!");
    }

    // Performance Monitor
    perf_mon_init();

    // Setup Telemetries (InfluxDB Line Protocol)
    // Topic: robot/odometry, Measurement: odometry, Interval: 5000ms
    tel_odometry = telemetry_create("robot/odometry", "odometry", 5000);
    
    // Topic: robot/telemetry, Measurement: system, Interval: 5000ms
    tel_system = telemetry_create("robot/telemetry", "system", 5000);

    // Subscriptions
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    mqtt_custom_client_subscribe("robot/cmd", 1);
    mqtt_custom_client_register_topic_callback("robot/cmd", mqtt_cmd_callback);

    ESP_LOGI(TAG, "Comms Task Running on Core %d", xPortGetCoreID());

    // 2. Main Loop
    TickType_t last_sample_time = 0;
    const TickType_t sample_interval_ticks = pdMS_TO_TICKS(1000); // 1s sampling rate

    while(1) {
        TickType_t now = xTaskGetTickCount();

        // Process Commands
        uint8_t command_buffer[CMD_QUEUE_ITEM_SIZE];
        if (xQueueReceive(command_queue, command_buffer, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Inter-core msg: %s", (char*)command_buffer);
        }

        // Sampling (Push data to telemetry buffers)
        // Note: Telemetries will flush asynchronously on their own timer
        if ((now - last_sample_time) >= sample_interval_ticks) {
             collect_sensor_data();
             last_sample_time = now;
        }

        // Servo Fun Animation: Simple sweep (independent of encoder/wheels)
        if (servo_handle) {
            int64_t current_us = esp_timer_get_time();
            int64_t time_since_last_us = current_us - servo_last_update_us;

            // Update servo every ~50ms (20Hz control rate)
            if (time_since_last_us >= 50000) {
                // Simple sweep: oscillate left-right
                servo_position += servo_direction;
                if (servo_position >= 1.0f) {
                    servo_position = 1.0f;
                    servo_direction = -0.02f;  // Reverse direction
                } else if (servo_position <= 0.0f) {
                    servo_position = 0.0f;
                    servo_direction = 0.02f;   // Reverse direction
                }

                // Move servo to new position
                servo_move_normalized(servo_handle, servo_position);
                servo_last_update_us = current_us;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void task_comms_cpu1_start(void)
{
    // Core 1, Priority 5
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 8192, NULL, 5, NULL, 1);
}
