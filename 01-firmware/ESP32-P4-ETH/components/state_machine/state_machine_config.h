#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

/**
 * Robot States
 * Configurable list of states
 */
typedef enum {
    STATE_INIT = 0,
    STATE_AUTONOMOUS,
    STATE_REMOTE_CONTROLLED,
    STATE_TELEMETRY_ONLY,
    STATE_WAITING_ORDERS,    // New initial test state
    STATE_MQTT_LOST_ERROR,
    STATE_SHUTDOWN,
    STATE_COUNT // Always last
} robot_state_t;

/**
 * Robot Modes
 * Configurable list of operation modes
 */
typedef enum {
    MODE_NONE = 0,
    MODE_REMOTE_DRIVE,
    MODE_TELEMETRY_STREAM,
    MODE_CALIBRATE_MOTORS,
    MODE_CALIBRATE_LINE,
    MODE_FOLLOW_LINE,
    MODE_COUNT // Always last
} robot_mode_t;

/**
 * Sensor Definitions (Bitmask)
 */
typedef enum {
    SENSOR_NONE = 0,
    SENSOR_TEST = (1 << 0),
    SENSOR_ODOMETRY = (1 << 1),
    SENSOR_LiDAR = (1 << 2),
    // Add more as needed
} robot_sensor_t;

/**
 * Transition Condition Types
 */
typedef enum {
    CONDITION_NONE = 0,
    CONDITION_MQTT_CONNECTED,
    CONDITION_MQTT_DISCONNECTED,
    CONDITION_TIMEOUT, // Requires data_val to be set to ms
    CONDITION_ALWAYS,
} transition_condition_t;

/**
 * State Transition Rule
 * Defines logic for moving between states
 */
typedef struct {
    robot_state_t from_state;
    robot_state_t to_state;
    transition_condition_t condition;
    uint32_t data_val;          // For timeout or other data
    robot_mode_t new_mode;      // Optional: switch mode on transition (MODE_NONE if no change)
    const char *event_name;     // For logging/publishing
} state_transition_rule_t;

/**
 * Mode Configuration
 * Defines requirements for each mode
 */
typedef struct {
    robot_mode_t mode;
    bool requires_mqtt;
    uint32_t sensor_mask;       // Bitmask of required sensors (robot_sensor_t)
} mode_config_t;

/**
 * Context for condition checking
 */
typedef struct {
    uint32_t current_state_time_ms;
    bool mqtt_connected;
    robot_mode_t current_mode;
} condition_context_t;

// Configuration Tables
extern const state_transition_rule_t transition_table[];
extern const size_t transition_table_size;

extern const mode_config_t mode_config_table[];
extern const size_t mode_config_table_size;

/**
 * Helpers to get config
 */
const mode_config_t* get_mode_config(robot_mode_t mode);
const char* get_state_name(robot_state_t state);
const char* get_mode_name(robot_mode_t mode);
