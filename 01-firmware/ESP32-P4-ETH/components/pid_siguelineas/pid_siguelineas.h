#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PID_SIGUELINEAS_NUM_SENSORS 5

typedef struct {
    gpio_num_t line_sensor_pins[PID_SIGUELINEAS_NUM_SENSORS];
    bool sensor_active_low;
    bool invert_steering;
    float kp;
    float ki;
    float kd;
    float kff;
    float dt_s;
    float integral_limit;
    int16_t base_speed_cmd;
    int16_t max_motor_cmd;
} pid_siguelineas_config_t;

typedef struct {
    bool line_detected;
    float line_error;
    float curvature_ff;
    float control_u;
    int16_t left_cmd;
    int16_t right_cmd;
} pid_siguelineas_debug_t;

esp_err_t pid_siguelineas_init(const pid_siguelineas_config_t *cfg);
void pid_siguelineas_reset(void);
esp_err_t pid_siguelineas_step(float curvature_ff, int16_t *left_cmd, int16_t *right_cmd, pid_siguelineas_debug_t *dbg);

#ifdef __cplusplus
}
#endif
