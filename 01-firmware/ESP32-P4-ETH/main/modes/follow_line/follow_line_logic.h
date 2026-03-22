#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_speed;
} follow_line_logic_config_t;

typedef struct {
    float line_position;
    bool line_detected;
    float base_speed; // Dynamic base speed (e.g. slowed down for curves)
} follow_line_logic_input_t;

typedef struct {
    float left_motor_speed;
    float right_motor_speed;

    // Diagnostics for telemetry
    float p_term;
    float i_term;
    float d_term;
    float raw_steering;
} follow_line_logic_output_t;

typedef struct follow_line_logic_context_t* follow_line_logic_handle_t;

esp_err_t follow_line_logic_create(const follow_line_logic_config_t* config, follow_line_logic_handle_t* out_handle);
esp_err_t follow_line_logic_destroy(follow_line_logic_handle_t handle);
esp_err_t follow_line_logic_update(follow_line_logic_handle_t handle, 
                                   const follow_line_logic_input_t* input, 
                                   follow_line_logic_output_t* out_output,
                                   float dt_s);

/**
 * @brief Update the internal configuration parameters for the follower.
 */
esp_err_t follow_line_logic_set_config(follow_line_logic_handle_t handle, const follow_line_logic_config_t* config);
