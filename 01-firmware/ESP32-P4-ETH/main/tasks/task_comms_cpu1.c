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
#include "ina226_sensor.h"
#include "pid_tuner.h"
#include "mqtt_api_responder.h"
#include "driver/gpio.h"
#include "ptp_client.h"

#include "task_comms_cpu1.h"

// =============================================================================
// Constants & Config
// =============================================================================
static const char *TAG = "comms_c1";

#define CMD_QUEUE_SIZE          32
#define CMD_QUEUE_ITEM_SIZE     256
#define POLL_INTERVAL_MS        10
#define CURVATURE_TOPIC         "robot/vision/curvature"

// =============================================================================
// Static Variables
// =============================================================================
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;

static telemetry_handle_t tel_odometry = NULL;
static telemetry_handle_t tel_system = NULL;

// =============================================================================
// Helper: Data Collection / Vision
// =============================================================================

/**
 * MQTT Callback for curvature updates from Vision System (e.g. Raspberry Pi)
 * Parses a float multiplier to adjust real-time base speed.
 */
static void mqtt_curvature_callback(const char *topic, int topic_len, const char *data, int data_len) 
{
    if (data == NULL || data_len <= 0) return;
    
    char payload[32] = {0};
    int copy_len = (data_len < 31) ? data_len : 31;
    memcpy(payload, data, copy_len);

    char *endptr = NULL;
    float val = strtof(payload, &endptr);
    if (endptr != payload) {
        shared_memory_t* shm = shared_memory_get();
        // Use a short timeout since this is high frequency
        if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            shm->vision_curvature_multiplier = val;
            xSemaphoreGive(shm->mutex);
        }
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

    ptp_client_init();

    // Initialize MQTT Client
    if (mqtt_custom_client_init() != ESP_OK) {
        ESP_LOGE(TAG, "MQTT client initialization failed");
        mqtt_initialized = false;
        vTaskDelete(NULL);
    }
    mqtt_initialized = true;


    perf_mon_init();
    
    // Setup Telemetry Batches
    tel_odometry = telemetry_create("robot/telemetry/odometry", "odometry", CONFIG_TELEMETRY_INTERVAL_ODOMETRY_MS);
    tel_system   = telemetry_create("robot/telemetry/system", "system", CONFIG_TELEMETRY_INTERVAL_SYSTEM_MS);

    vTaskDelay(pdMS_TO_TICKS(1000)); 
    
    // Register Asynchronous Responders
    pid_tuner_init();
    pid_tuner_register_callback();
    pid_tuner_subscribe();
    mqtt_api_responder_init();
    mqtt_api_responder_subscribe();

    // Register & Subscribe Vision Curvature
    mqtt_custom_client_register_topic_callback(CURVATURE_TOPIC, mqtt_curvature_callback);
    if (mqtt_custom_client_is_connected()) {
        mqtt_custom_client_subscribe(CURVATURE_TOPIC, 0);
    }

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
                 pid_tuner_subscribe();
                 mqtt_api_responder_subscribe();
                 mqtt_custom_client_subscribe(CURVATURE_TOPIC, 0);
             } else {
                 shared_memory_set_mqtt_connected(false);
             }

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
    xTaskCreatePinnedToCore(task_comms_cpu1, "comms_cpu1", 12288, NULL, 10, NULL, 1);
}
