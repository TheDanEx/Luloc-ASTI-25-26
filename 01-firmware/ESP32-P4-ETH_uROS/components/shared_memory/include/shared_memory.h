#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// Data Structures (Telemetry Groups)
// =============================================================================

// Fast Telemetry (10Hz): Critical for control loop monitoring
typedef struct {
    float line_raw[8];
    float line_norm[8];
    float pid_error;
    float pid_setpoint;
    float pid_output;
    float pid_p;
    float pid_i;
    float pid_d;
} robot_telemetry_fast_t;

// Slow Telemetry (1Hz): State and calibration
typedef struct {
    float odom_x;
    float odom_y;
    float odom_theta;
    float odom_lin;
    float odom_ang;
    float calib_min[8];
    float calib_max[8];
    float battery_v;
    uint32_t current_mode;
    uint32_t uptime_s;
    float latency_ms;
    float jitter_ms;
} robot_telemetry_slow_t;

// Commands (ROS -> ESP32)
typedef struct {
    uint8_t new_mode;
} robot_mode_cmd_t;

typedef struct {
    float linear_x;
    float angular_z;
} robot_twist_cmd_t;

// =============================================================================
// Public API
// =============================================================================

esp_err_t shared_memory_init(void);

// Producers (Core 1)
esp_err_t shared_memory_push_telemetry_fast(const robot_telemetry_fast_t *data);
esp_err_t shared_memory_push_telemetry_slow(const robot_telemetry_slow_t *data);

// Consumers (Core 1)
esp_err_t shared_memory_get_mode_cmd(robot_mode_cmd_t *cmd);
esp_err_t shared_memory_get_twist_cmd(robot_twist_cmd_t *cmd);

// Consumers (Core 0)
esp_err_t shared_memory_get_telemetry_fast(robot_telemetry_fast_t *data);
esp_err_t shared_memory_get_telemetry_slow(robot_telemetry_slow_t *data);

// Producers (Core 0)
esp_err_t shared_memory_put_mode_cmd(const robot_mode_cmd_t *cmd);
esp_err_t shared_memory_put_twist_cmd(const robot_twist_cmd_t *cmd);

#endif // SHARED_MEMORY_H
