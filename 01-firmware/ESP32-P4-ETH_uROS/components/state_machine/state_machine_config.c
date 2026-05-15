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
        .condition = CONDITION_TIMEOUT, 
        .new_mode = MODE_TELEMETRY_STREAM,
        .event_name = "STATE_WAITING_ORDERS" 
    },

    // FROM STATE_AUTONOMOUS
    { 
        .from_state = STATE_AUTONOMOUS, 
        .to_state = STATE_TELEMETRY_ONLY, 
        .condition = CONDITION_TIMEOUT, 
        .new_mode = MODE_NONE, // Keep current autonomous mode
        .event_name = "STATE_TELEMETRY_ONLY" 
    },

};

const size_t transition_table_size = sizeof(transition_table) / sizeof(transition_table[0]);

/*
 * Mode Configuration Table
 */
const mode_config_t mode_config_table[] = {
    { .mode = MODE_NONE,               .sensor_mask = SENSOR_NONE },
    { .mode = MODE_AUTONOMOUS_PATH,    .sensor_mask = SENSOR_ODOMETRY | SENSOR_LiDAR | SENSOR_LINE },
    { .mode = MODE_AUTONOMOUS_OBSTACLE,.sensor_mask = SENSOR_LiDAR },
    { .mode = MODE_REMOTE_DRIVE,       .sensor_mask = SENSOR_ODOMETRY },
    // Telemetry stream uses SENSOR_TEST for this initial test
    { .mode = MODE_TELEMETRY_STREAM,   .sensor_mask = SENSOR_TEST },
    // Calibration Modes
    { .mode = MODE_CALIBRATE_MOTORS,   .sensor_mask = SENSOR_ODOMETRY },
    { .mode = MODE_CALIBRATE_LINE,     .sensor_mask = SENSOR_ODOMETRY | SENSOR_LINE },
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
        "SUMO",
        "SHUTDOWN"
    };
    if (state < STATE_COUNT) return names[state];
    return "UNKNOWN";
}

const char* get_mode_name(robot_mode_t mode) {
    // Basic names, could be in a table if needed
    switch(mode) {
        case MODE_NONE: return "NONE";
        case MODE_AUTONOMOUS_PATH: return "AUTO_PATH";
        case MODE_AUTONOMOUS_OBSTACLE: return "AUTO_OBSTACLE";
        case MODE_REMOTE_DRIVE: return "REMOTE";
        case MODE_TELEMETRY_STREAM: return "TELEMETRY";
        case MODE_CALIBRATE_MOTORS: return "CALIB_MOTORS";
        case MODE_CALIBRATE_LINE: return "CALIB_LINE";
        case MODE_SUMO: return "SUMO";
        default: return "UNKNOWN";
    }
}
