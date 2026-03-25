#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "state_machine_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    robot_mode_t mode;
    uint32_t raw[8];
    float norm[8];
    float position;
    float kp;
    float ki;
    float kd;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float pid_out;
    float ff_val;
    float target_speed_l;
    float target_speed_r;
} line_follower_telemetry_t;

/**
 * @brief Initialize the telemetry task and queue
 */
void task_telemetry_start(void);

/**
 * @brief Send a telemetry point to the queue (Non-blocking)
 * 
 * @param data Pointer to telemetry data
 */
void task_telemetry_send(const line_follower_telemetry_t *data);

#ifdef __cplusplus
}
#endif
