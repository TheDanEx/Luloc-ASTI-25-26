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

// =============================================================================
// Static Variables
// =============================================================================
static QueueHandle_t command_queue = NULL;
static volatile bool mqtt_initialized = false;

static telemetry_handle_t tel_odometry = NULL;
static telemetry_handle_t tel_system = NULL;
static telemetry_handle_t tel_line = NULL;

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

    // Wheel Odometry Sync (Reading from SHM populated by CPU0)
    float speed_l, dist_l, speed_r, dist_r;
    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    speed_l = shm->sensors.motor_speed_left;
    dist_l  = shm->sensors.motor_distance_left;
    speed_r = shm->sensors.motor_speed_right;
    dist_r  = shm->sensors.motor_distance_right;
    xSemaphoreGive(shm->mutex);

    telemetry_add_float(tel_odometry, "velIZ", speed_l);
    telemetry_add_float(tel_odometry, "posIZ", dist_l);
    telemetry_add_float(tel_odometry, "velDR", speed_r);
    telemetry_add_float(tel_odometry, "posDR", dist_r);
    telemetry_commit_point(tel_odometry);

    // Line Sensor Telemetry
    float err_line, norm[8], kp, ki, kd, target_l, target_r;
    uint16_t min_vals[8], max_vals[8];
    bool detected, is_cal;

    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    err_line = shm->sensors.line_position;
    detected = shm->sensors.line_detected;
    is_cal   = shm->sensors.line_is_calibrated;
    target_l = shm->sensors.target_speed_left;
    target_r = shm->sensors.target_speed_right;
    kp = shm->line_pid.kp;
    ki = shm->line_pid.ki;
    kd = shm->line_pid.kd;
    memcpy(norm, shm->sensors.line_norm, 8 * sizeof(float));
    memcpy(min_vals, shm->sensors.line_min, 8 * sizeof(uint16_t));
    memcpy(max_vals, shm->sensors.line_max, 8 * sizeof(uint16_t));
    xSemaphoreGive(shm->mutex);

    telemetry_add_float(tel_line, "err", err_line);
    telemetry_add_float(tel_line, "target_l", target_l);
    telemetry_add_float(tel_line, "target_r", target_r);
    telemetry_add_int(tel_line, "det", detected ? 1 : 0);
    telemetry_add_int(tel_line, "is_cal", is_cal ? 1 : 0);
    telemetry_add_float(tel_line, "kp", kp);
    telemetry_add_float(tel_line, "ki", ki);
    telemetry_add_float(tel_line, "kd", kd);

    // Add individual sensor values
    char key[8];
    for (int i = 0; i < 8; i++) {
        snprintf(key, sizeof(key), "s%d", i);
        telemetry_add_float(tel_line, key, norm[i]);
        snprintf(key, sizeof(key), "min%d", i);
        telemetry_add_int(tel_line, key, min_vals[i]);
        snprintf(key, sizeof(key), "max%d", i);
        telemetry_add_int(tel_line, key, max_vals[i]);
    }
    telemetry_commit_point(tel_line);

    // System Metrics & State
    test_sensor_data_t sys_data;
    if (test_sensor_read(&sys_data) == ESP_OK) {
        telemetry_add_int(tel_system, "uptime_sec", sys_data.uptime_sec);
        telemetry_add_int(tel_system, "uptime_ms", sys_data.uptime_ms);
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
    tel_line     = telemetry_create("robot/telemetry/line", "line_sensor", 50); // 20Hz Debug

    vTaskDelay(pdMS_TO_TICKS(1000)); 
    
    // Register Asynchronous Responders
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
                 pid_tuner_subscribe();
                 mqtt_api_responder_subscribe();
             } else {
                 shared_memory_set_mqtt_connected(false);
             }

             last_sampling_tick = current_tick;
        }

        // 4. High-Frequency Sampling Call
        collect_high_freq_sensor_data();

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
