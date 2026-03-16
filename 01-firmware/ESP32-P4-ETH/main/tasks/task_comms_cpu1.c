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
#include "ina226_sensor.h"
#include "curvature_feedforward.h"
#include "pid_tuner.h"
#include "mqtt_api_responder.h"
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
#define ENC_WHEEL_DIA_M         0.068f  // 6.3 cm
#define ENC_GEAR_RATIO          21.3f   // 73:1 reduction

// =============================================================================
// Static Variables
// =============================================================================
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;
static encoder_sensor_handle_t encoder_handle = NULL;

static telemetry_handle_t tel_odometry = NULL;
static telemetry_handle_t tel_system = NULL;
static telemetry_handle_t tel_power = NULL;

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
    // String commands are now deprecated.
    // All synchronous interactions pass through the JSON API (robot/api/set).
    ESP_LOGW(TAG, "Legacy string cmd ignored: %s. Use JSON API.", cmd_str);
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

    if (curvature_feedforward_has_value()) {
        telemetry_add_float(tel_system, "curvatura_ff", curvature_feedforward_get_value());
    }

    // 3. Power Sensor Data (INA226/INA219)
    float bus_voltage_mv = 0;
    float current_ma = 0;
    float power_mw = 0;

    if (ina226_sensor_read_bus_voltage_mv(&bus_voltage_mv) == ESP_OK &&
        ina226_sensor_read_current_ma(&current_ma) == ESP_OK &&
        ina226_sensor_read_power_mw(&power_mw) == ESP_OK) {
        
        // Save to shared memory for motor control
        shared_memory_t* shm = shared_memory_get();
        xSemaphoreTake(shm->mutex, portMAX_DELAY);
        shm->sensors.robot_current = bus_voltage_mv; // Naming is a bit mixed up, but this stores voltage
        xSemaphoreGive(shm->mutex);

        // Calculate power and push to telemetry
        float cur_a = current_ma / 1000.0f;
        float vol_v = bus_voltage_mv / 1000.0f;
        float pwr_w = power_mw / 1000.0f;

        telemetry_add_float(tel_power, "v_bat", vol_v);
        telemetry_add_float(tel_power, "a_bat", cur_a);
        telemetry_add_float(tel_power, "w_bat", pwr_w);
    }
    
    // 4. Performance Data
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
    ESP_LOGI(TAG, "Comms task started, waiting for Ethernet link...");

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

    // Performance Monitor
    perf_mon_init();

    // Test Sensor (uptime)
    test_sensor_init();

    // Setup Telemetries (InfluxDB Line Protocol)
    // Topic: robot/odometry, Measurement: odometry, Interval: 5000ms
    tel_odometry = telemetry_create("robot/odometry", "odometry", 5000);
    
    // Topic: robot/telemetry, Measurement: system, Interval: 5000ms
    tel_system = telemetry_create("robot/telemetry", "system", 5000);

    // Topic: robot/power, Measurement: power, Interval: 1000ms
    tel_power = telemetry_create("robot/power", "power", 1000);

    // Sensors
    ina226_sensor_init();

    // Subscriptions
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    mqtt_custom_client_subscribe("robot/cmd", 1);
    mqtt_custom_client_register_topic_callback("robot/cmd", mqtt_cmd_callback);
    
    curvature_feedforward_register_callback();
    curvature_feedforward_subscribe();

    pid_tuner_init();
    pid_tuner_register_callback();
    pid_tuner_subscribe();

    mqtt_api_responder_init();
    mqtt_api_responder_subscribe();

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
             if (mqtt_custom_client_is_connected()) {
                 shared_memory_set_mqtt_connected(true);
                 if (curvature_feedforward_subscribe() != ESP_OK) {
                     // Keep retrying silently while broker/client stabilizes.
                 }
                 if (pid_tuner_subscribe() != ESP_OK) {
                     // Retry logic natively handled
                 }
                 if (mqtt_api_responder_subscribe() != ESP_OK) {
                     // Retry logic natively handled
                 }
             } else {
                 shared_memory_set_mqtt_connected(false);
             }

             collect_sensor_data();
             last_sample_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void task_comms_cpu1_start(void)
{
    // Core 1, Priority 5
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 8192, NULL, 10, NULL, 1);
}
