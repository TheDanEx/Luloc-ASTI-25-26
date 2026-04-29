/**
 * @file shared_memory.h
 * @brief Inter-Process Communication (IPC) manager for Core 0 and Core 1.
 * 
 * This component handles the data exchange between the Communications Core (Core 0)
 * and the Real-Time Control Core (Core 1). It uses non-blocking FreeRTOS primitives
 * to ensure Core 1 determinism.
 */

#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Line sensor readings (Core 1 -> Core 0)
 */
typedef struct {
    float raw[8];
    float normalized[8];
    uint64_t timestamp_us;
} robot_line_sensors_t;

/**
 * @brief Calibration bounds (Core 1 -> Core 0)
 */
typedef struct {
    float min[8];
    float max[8];
} robot_calibration_t;

/**
 * @brief PID controller data (Core 1 -> Core 0)
 */
typedef struct {
    float error;
    float setpoint;
    float output;
    float p_term;
    float i_term;
    float d_term;
} robot_pid_data_t;

/**
 * @brief Odometry data (Core 1 -> Core 0)
 */
typedef struct {
    float pos_x;
    float pos_y;
    float theta;
    float vel_linear;
    float vel_angular;
} robot_odometry_t;

/**
 * @brief System state from Core 1 to Core 0 (Fast Producer -> Slow Consumer)
 */
typedef struct {
    int8_t current_mode;
    float battery_voltage;
    float network_latency_ms;
    float network_jitter_ms;
    uint32_t uptime_s;
} robot_state_t;

/**
 * @brief Mode command from Core 0 to Core 1 (Slow Producer -> Fast Consumer)
 */
typedef struct {
    int8_t new_mode;
} robot_mode_cmd_t;

/**
 * @brief Twist (velocity) command from Core 0 to Core 1 (Slow Producer -> Fast Consumer)
 */
typedef struct {
    float linear_x;
    float linear_y;
    float angular_z;
} robot_twist_cmd_t;

// =============================================================================
// Public API
// =============================================================================

esp_err_t shared_memory_init(void);

// --- Core 1 -> Core 0 (Telemetry & State) ---

void shared_memory_push_line_sensors(const robot_line_sensors_t *data);
esp_err_t shared_memory_get_line_sensors(robot_line_sensors_t *data);

void shared_memory_push_calibration(const robot_calibration_t *data);
esp_err_t shared_memory_get_calibration(robot_calibration_t *data);

void shared_memory_push_pid_data(const robot_pid_data_t *data);
esp_err_t shared_memory_get_pid_data(robot_pid_data_t *data);

void shared_memory_push_odometry(const robot_odometry_t *data);
esp_err_t shared_memory_get_odometry(robot_odometry_t *data);

void shared_memory_push_state(const robot_state_t *data);
esp_err_t shared_memory_get_state(robot_state_t *data);

// --- Core 0 -> Core 1 (Commands) ---

void shared_memory_push_mode_cmd(const robot_mode_cmd_t *cmd);
esp_err_t shared_memory_get_mode_cmd(robot_mode_cmd_t *cmd);

void shared_memory_push_twist_cmd(const robot_twist_cmd_t *cmd);
esp_err_t shared_memory_get_twist_cmd(robot_twist_cmd_t *cmd);

#endif // SHARED_MEMORY_H
