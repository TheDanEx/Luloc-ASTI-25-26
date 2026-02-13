#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Pines físicos del ESP32 conectados a IN1/IN2 del DRV8874
    gpio_num_t in1;
    gpio_num_t in2;
    // MCPWM: unidad + timer usados para este motor
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
} motor_hw_mcpwm_t;

typedef struct {
    motor_hw_mcpwm_t left;
    motor_hw_mcpwm_t right;

    // Opcional: nSLEEP (si no lo usas, pon GPIO_NUM_NC)
    gpio_num_t nsleep;

    // Frecuencia PWM (Hz)
    uint32_t pwm_hz;

    // Zona muerta en [-1000..1000]
    int16_t deadband;

    // Si true: al stop hace brake (IN1=IN2=1). Si false: coast (IN1=IN2=0)
    bool brake_on_stop;
} motor_driver_mcpwm_t;

bool motor_mcpwm_init(motor_driver_mcpwm_t *m);

/**
 * @brief Set de potencia por motor (rango -1000..1000)
 */
void motor_mcpwm_set(motor_driver_mcpwm_t *m, int16_t left, int16_t right);

void motor_mcpwm_stop(motor_driver_mcpwm_t *m);
void motor_mcpwm_brake(motor_driver_mcpwm_t *m);
void motor_mcpwm_coast(motor_driver_mcpwm_t *m);
void motor_mcpwm_sleep(motor_driver_mcpwm_t *m, bool sleep);

#ifdef __cplusplus
}
#endif
