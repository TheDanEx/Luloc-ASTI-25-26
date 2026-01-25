#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * System State Machine - CPU0 (Real-Time Control)
 * 
 * Manages operational modes based on communication status:
 * - AUTONOMOUS: No telemetry required, runs without MQTT
 * - REMOTE_CONTROLLED: Requires MQTT for commands and telemetry
 * - TELEMETRY_ONLY: Streams data to broker, autonomous control
 * - ERROR: MQTT lost during operation requiring it - STOP
 * - INIT: Starting up
 */

#include "state_machine_config.h"

// Enums robot_state_t and robot_mode_t are now in state_machine_config.h

/**
 * System state context
 */
typedef struct {
    robot_state_t current_state;
    robot_state_t previous_state;
    robot_mode_t current_mode;
    bool mqtt_connected;         // MQTT connection status from CPU1
    bool mode_requires_mqtt;     // True if current mode needs MQTT
    uint32_t state_time_ms;      // Time in current state
    uint32_t error_code;         // Error details
} robot_state_context_t;

/**
 * Initialize state machine
 */
void state_machine_init(void);

/**
 * Update state machine (call from task_rtcontrol_cpu0)
 * Returns current state
 */
robot_state_t state_machine_update(void);

/**
 * Notify state machine of MQTT status change
 */
void state_machine_notify_mqtt_status(bool connected);

/**
 * Request state transition to new mode
 * Returns true if transition allowed, false if blocked
 */
bool state_machine_request_mode(robot_mode_t new_mode);

/**
 * Get current state context
 */
robot_state_context_t* state_machine_get_context(void);

/**
 * Check if current mode allows autonomous operation
 */
bool state_machine_is_autonomous_safe(void);

/**
 * Check if current mode requires MQTT
 */
bool state_machine_requires_mqtt(void);

/**
 * Get mask of active sensors for current mode
 */
uint32_t state_machine_get_active_sensors(void);

/**
 * Get readable state name
 */
const char* state_machine_get_state_name(robot_state_t state);
