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

#define ENCODER_LEFT_PIN_A      33
#define ENCODER_LEFT_PIN_B      46
#define ENCODER_RIGHT_PIN_A     27
#define ENCODER_RIGHT_PIN_B     32
#define ENCODER_PPR             11
#define WHEEL_DIAMETER_M        0.068f
#define GEAR_RATIO              21.3f

// =============================================================================
// Static Variables
// =============================================================================
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;
static encoder_sensor_handle_t encoder_left = NULL;
static encoder_sensor_handle_t encoder_right = NULL;

static telemetry_handle_t tel_odometry = NULL;
static telemetry_handle_t tel_system = NULL;

// =============================================================================
// Helper: Data Collection
// =============================================================================

/**
 * Capture high-frequency metrics from local sensors.
 * This function handles odometry calculations and system telemetry 
 * batching before committing points to the asynchronous reporter.
 */
static void collect_high_freq_sensor_data(void)
{
    shared_memory_t* shm = shared_memory_get();

    // Wheel Odometry Sync
    if (encoder_left) {
        float speed = encoder_sensor_get_speed(encoder_left);
        float distance = encoder_sensor_get_distance(encoder_left);
        telemetry_add_float(tel_odometry, "velIZ", speed);
        telemetry_add_float(tel_odometry, "posIZ", distance);
        
        xSemaphoreTake(shm->mutex, portMAX_DELAY);
        shm->sensors.motor_speed_left = speed;
        shm->sensors.motor_distance_left = distance;
        xSemaphoreGive(shm->mutex);
    }

    if (encoder_right) {
        float speed = encoder_sensor_get_speed(encoder_right);
        float distance = encoder_sensor_get_distance(encoder_right);
        telemetry_add_float(tel_odometry, "velDR", speed);
        telemetry_add_float(tel_odometry, "posDR", distance);
        
        xSemaphoreTake(shm->mutex, portMAX_DELAY);
        shm->sensors.motor_speed_right = speed;
        shm->sensors.motor_distance_right = distance;
        xSemaphoreGive(shm->mutex);
    }

    if (encoder_left || encoder_right) {
        telemetry_commit_point(tel_odometry);
    }

    // System Metrics & State
    test_sensor_data_t sys_data;
    if (test_sensor_read(&sys_data) == ESP_OK) {
        telemetry_add_int(tel_system, "uptime_sec", sys_data.uptime_sec);
        telemetry_add_int(tel_system, "uptime_ms", sys_data.uptime_ms);
        
        if (curvature_feedforward_has_value()) {
            telemetry_add_float(tel_system, "curvatura_ff", curvature_feedforward_get_value());
        }
        telemetry_commit_point(tel_system);
    }
}

// =============================================================================
// Public API: Lifecycle
// =============================================================================

void task_comms_cpu1_init_queue(void)
{
    if (command_queue == NULL) {
        command_queue = xQueueCreate(CMD_QUEUE_SIZE, CMD_QUEUE_ITEM_SIZE);
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
// Main Task Implementation
// =============================================================================

/**
 * Communications and Telemetry Hub (CPU 1).
 * Responsibilities:
 * 1. Maintain MQTT connectivity and topic subscriptions.
 * 2. Manage high-level responders (API, PID Tuning).
 * 3. Sample and batch high-frequency sensor telemetry (Odometry).
 * 4. Coordinate inter-core command routing.
 */
static void task_comms_cpu1(void *arg)
{
    task_comms_cpu1_init_queue();
    
    // Block until network is physically ready
    while (!ethernet_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Initialize MQTT Client
    if (mqtt_custom_client_init() != ESP_OK) {
        ESP_LOGE(TAG, "MQTT client initialization failed");
        mqtt_initialized = false;
        vTaskDelete(NULL);
    }
    mqtt_initialized = true;

    // Initialize Wheel Encoders
    encoder_sensor_config_t enc_l_cfg = {
        .pin_a = ENCODER_LEFT_PIN_A,
        .pin_b = ENCODER_LEFT_PIN_B,
        .ppr = ENCODER_PPR,
        .wheel_diameter_m = WHEEL_DIAMETER_M,
        .gear_ratio = GEAR_RATIO,
        .reverse_direction = false
    };
    encoder_left = encoder_sensor_init(&enc_l_cfg);

    encoder_sensor_config_t enc_r_cfg = {
        .pin_a = ENCODER_RIGHT_PIN_A,
        .pin_b = ENCODER_RIGHT_PIN_B,
        .ppr = ENCODER_PPR,
        .wheel_diameter_m = WHEEL_DIAMETER_M,
        .gear_ratio = GEAR_RATIO,
        .reverse_direction = false
    };
    encoder_right = encoder_sensor_init(&enc_r_cfg);

    perf_mon_init();
    
    // Setup Telemetry Batches
    tel_odometry = telemetry_create("robot/telemetry/odometry", "odometry", 5000);
    tel_system   = telemetry_create("robot/telemetry/system", "system", 5000);

    vTaskDelay(pdMS_TO_TICKS(1000)); 
    
    // Register Asynchronous Responders
    curvature_feedforward_register_callback();
    curvature_feedforward_subscribe();
    pid_tuner_init();
    pid_tuner_register_callback();
    pid_tuner_subscribe();
    mqtt_api_responder_init();
    mqtt_api_responder_subscribe();

    TickType_t last_sampling_tick = 0;
    bool last_mqtt_conn = false;

    while(1) {
        TickType_t current_tick = xTaskGetTickCount();

        // 1. MQTT Connectivity Sync
        bool current_mqtt_conn = mqtt_custom_client_is_connected();
        if (current_mqtt_conn != last_mqtt_conn) {
            state_machine_notify_mqtt_status(current_mqtt_conn);
            last_mqtt_conn = current_mqtt_conn;
        }

        // 2. Inter-core Command Handling
        uint8_t intercore_msg[CMD_QUEUE_ITEM_SIZE];
        if (xQueueReceive(command_queue, intercore_msg, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Inter-core message: %s", (char*)intercore_msg);
        }

        // 3. Periodic Sampling Loop (1Hz)
        if ((current_tick - last_sampling_tick) >= pdMS_TO_TICKS(1000)) {
             if (mqtt_custom_client_is_connected()) {
                 shared_memory_set_mqtt_connected(true);
                 // Resubscribe if connection was dropped and restored
                 curvature_feedforward_subscribe();
                 pid_tuner_subscribe();
                 mqtt_api_responder_subscribe();
             } else {
                 shared_memory_set_mqtt_connected(false);
             }
             collect_high_freq_sensor_data();
             last_sampling_tick = current_tick;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

// =============================================================================
// Task Start Wrapper
// =============================================================================

void task_comms_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 8192, NULL, 10, NULL, 1);
}
