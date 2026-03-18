#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Shared memory structure for inter-core communication
 * CPU0: Writes sensor data and control state
 * CPU1: Reads for telemetry packing and publishing
 */

typedef struct {
    // Sensor data from CPU0 (motor, encoder, etc.)
    float motor_speed;          // RPM or rad/s
    float motor_current;        // mA
    float battery_voltage;      // mV
    float temperature;          // °C
    int32_t encoder_count;      // Ticks
    uint32_t timestamp_ms;      // Local timestamp
} robot_sensor_data_t;

typedef struct {
    // Commands received by CPU1 to execute on CPU0
    uint8_t command_type;       // Command ID
    int16_t param1;             // Parameter 1 (speed, angle, etc.)
    int16_t param2;             // Parameter 2
    uint32_t timestamp_ms;      // Command timestamp
} robot_command_t;

typedef struct {
    // Core shared state
    robot_sensor_data_t sensors;
    robot_command_t last_command;
    uint32_t heartbeat_cpu0;    // CPU0 heartbeat counter
    uint32_t heartbeat_cpu1;    // CPU1 heartbeat counter
    bool cpu0_alive;
    bool cpu1_alive;
    bool mqtt_connected;        // CPU1 reports MQTT connection status to CPU0
    float curvature_ff;         // Feedforward curvature from camera/MQTT
    uint32_t curvature_ts_ms;   // Timestamp of curvature_ff
    bool curvature_valid;
    SemaphoreHandle_t mutex;    // Protect concurrent access
} shared_memory_t;

/**
 * Initialize shared memory and synchronization primitives
 */
void shared_memory_init(void);

/**
 * Write sensor data from CPU0 (non-blocking with timeout)
 */
bool shared_memory_write_sensors(const robot_sensor_data_t *data, TickType_t timeout);

/**
 * Read sensor data from CPU1 (non-blocking with timeout)
 */
bool shared_memory_read_sensors(robot_sensor_data_t *data, TickType_t timeout);

/**
 * Write command from CPU1 for CPU0 to execute
 */
bool shared_memory_write_command(const robot_command_t *cmd, TickType_t timeout);

/**
 * Read command from CPU0
 */
bool shared_memory_read_command(robot_command_t *cmd, TickType_t timeout);

/**
 * Update heartbeat counters
 */
void shared_memory_heartbeat_cpu0(void);
void shared_memory_heartbeat_cpu1(void);

/**
 * MQTT connection status (CPU1 → CPU0)
 */
void shared_memory_set_mqtt_connected(bool connected);
bool shared_memory_get_mqtt_connected(void);

/**
 * Camera curvature feedforward (CPU1 -> CPU0)
 */
void shared_memory_set_curvature_ff(float curvature, uint32_t timestamp_ms);
bool shared_memory_get_curvature_ff(float *curvature, uint32_t *timestamp_ms, TickType_t timeout);

/**
 * Get reference to shared memory (advanced use)
 */
shared_memory_t* shared_memory_get(void);
