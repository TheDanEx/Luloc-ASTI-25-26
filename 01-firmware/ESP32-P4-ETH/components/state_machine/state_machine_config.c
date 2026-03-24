#include "state_machine_config.h"
#include <stddef.h>

/*
 * Transition Table
 * ORDER MATTERS: The first matching rule will be taken.
 */
const state_transition_rule_t transition_table[] = {
    // FROM STATE_INIT
    // Path for initial test: Just wait for orders and stream basic data
    { 
        .from_state = STATE_INIT, 
        .to_state = STATE_WAITING_ORDERS, 
        .condition = CONDITION_MQTT_CONNECTED, 
        .new_mode = MODE_TELEMETRY_STREAM,
        .event_name = "STATE_WAITING_ORDERS" 
    },

    // FROM STATE_WAITING_ORDERS
    // Allow going to Remote Control if requested/logic permits (or just stay here for test)
    { 
        .from_state = STATE_WAITING_ORDERS, 
        .to_state = STATE_MQTT_LOST_ERROR, 
        .condition = CONDITION_MQTT_DISCONNECTED, 
        .new_mode = MODE_NONE,
        .event_name = "STATE_MQTT_LOST_ERROR - MQTT_DISCONNECTED" 
    },

    // FROM STATE_AUTONOMOUS
    { 
        .from_state = STATE_AUTONOMOUS, 
        .to_state = STATE_TELEMETRY_ONLY, 
        .condition = CONDITION_MQTT_CONNECTED, 
        .new_mode = MODE_NONE, // Keep current autonomous mode
        .event_name = "STATE_TELEMETRY_ONLY" 
    },

    // FROM STATE_REMOTE_CONTROLLED
    { 
        .from_state = STATE_REMOTE_CONTROLLED, 
        .to_state = STATE_MQTT_LOST_ERROR, 
        .condition = CONDITION_MQTT_DISCONNECTED, 
        .new_mode = MODE_NONE,
        .event_name = "STATE_MQTT_LOST_ERROR - MQTT_DISCONNECTED" 
    },

    // FROM STATE_TELEMETRY_ONLY
    { 
        .from_state = STATE_TELEMETRY_ONLY, 
        .to_state = STATE_MQTT_LOST_ERROR, 
        .condition = CONDITION_MQTT_DISCONNECTED, 
        .new_mode = MODE_NONE,
        .event_name = "STATE_MQTT_LOST_ERROR - TELEMETRY_FAILED" 
    },

    // FROM STATE_MQTT_LOST_ERROR
    { 
        .from_state = STATE_MQTT_LOST_ERROR, 
        .to_state = STATE_WAITING_ORDERS, // Return to waiting orders on recovery
        .condition = CONDITION_TIMEOUT, 
        .data_val = 5000, 
        .new_mode = MODE_TELEMETRY_STREAM,
        .event_name = "STATE_WAITING_ORDERS - RECOVERED" 
    },
};

const size_t transition_table_size = sizeof(transition_table) / sizeof(transition_table[0]);

/*
 * Mode Configuration Table
 */
const mode_config_t mode_config_table[] = {
    { .mode = MODE_NONE,                .requires_mqtt = false, .sensor_mask = SENSOR_NONE },
    { .mode = MODE_REMOTE_DRIVE,        .requires_mqtt = true,  .sensor_mask = SENSOR_ODOMETRY },
    // Telemetry stream uses SENSOR_TEST for this initial test
    { .mode = MODE_TELEMETRY_STREAM,    .requires_mqtt = true,  .sensor_mask = SENSOR_TEST },
    // Calibration Modes
    { .mode = MODE_CALIBRATE_MOTORS,    .requires_mqtt = true,  .sensor_mask = SENSOR_ODOMETRY },
    { .mode = MODE_CALIBRATE_LINE,      .requires_mqtt = true,  .sensor_mask = SENSOR_ODOMETRY },
    { .mode = MODE_FOLLOW_LINE,         .requires_mqtt = false, .sensor_mask = SENSOR_ODOMETRY },
};


const size_t mode_config_table_size = sizeof(mode_config_table) / sizeof(mode_config_table[0]);

const mode_config_t* get_mode_config(robot_mode_t mode) {
    for (size_t i = 0; i < mode_config_table_size; i++) {
        if (mode_config_table[i].mode == mode) {
            return &mode_config_table[i];
        }
    }
    return NULL;
}

const char* get_state_name(robot_state_t state) {
    static const char *names[] = {
        "INIT",
        "AUTONOMOUS",
        "REMOTE_CONTROLLED",
        "TELEMETRY_ONLY",
        "WAITING_ORDERS",
        "MQTT_LOST_ERROR",
        "SHUTDOWN"
    };
    if (state < STATE_COUNT) return names[state];
    return "UNKNOWN";
}

const char* get_mode_name(robot_mode_t mode) {
    // Basic names, could be in a table if needed
    switch(mode) {
        case MODE_NONE: return "NONE";
        case MODE_REMOTE_DRIVE: return "REMOTE";
        case MODE_TELEMETRY_STREAM: return "TELEMETRY";
        case MODE_CALIBRATE_MOTORS: return "CALIB_MOTORS";
        case MODE_CALIBRATE_LINE: return "CALIB_LINE";
        case MODE_FOLLOW_LINE: return "FOLLOW_LINE";
        default: return "UNKNOWN";
    }
}
