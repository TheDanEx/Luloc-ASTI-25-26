#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Pines físicos hacia IN1/IN2 del DRV8874
    gpio_num_t in1;
    gpio_num_t in2;

    // Recursos MCPWM para este motor
    mcpwm_oper_handle_t oper;

    mcpwm_cmpr_handle_t cmpr_in1;
    mcpwm_cmpr_handle_t cmpr_in2;

    mcpwm_gen_handle_t gen_in1;
    mcpwm_gen_handle_t gen_in2;
} motor_hw_mcpwm_t;

typedef struct {
    motor_hw_mcpwm_t left;
    motor_hw_mcpwm_t right;

    // Opcional: nSLEEP del DRV8874 (si no lo controlas, pon GPIO_NUM_NC)
    gpio_num_t nsleep;

    // PWM
    uint32_t pwm_hz;          // ej 20000
    uint32_t resolution_hz;   // ej 10*1000*1000 (10 MHz)
    uint32_t period_ticks;    // resolution_hz / pwm_hz

    // Lógica
    int16_t deadband;         // rango 0..1000
    bool brake_on_stop;

    // MCPWM timer compartido
    mcpwm_timer_handle_t timer;
} motor_driver_mcpwm_t;

esp_err_t motor_mcpwm_init(motor_driver_mcpwm_t *m);

void motor_mcpwm_set(motor_driver_mcpwm_t *m, int16_t left, int16_t right);
void motor_mcpwm_stop(motor_driver_mcpwm_t *m);
void motor_mcpwm_coast(motor_driver_mcpwm_t *m);
void motor_mcpwm_brake(motor_driver_mcpwm_t *m);

void motor_mcpwm_sleep(motor_driver_mcpwm_t *m, bool sleep);

#ifdef __cplusplus
}
#endif
