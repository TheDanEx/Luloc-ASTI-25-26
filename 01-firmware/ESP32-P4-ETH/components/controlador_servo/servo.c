#include "servo.h"
#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_FREQ_HZ 50
#define LEDC_RESOLUTION LEDC_TIMER_16_BIT
#define LEDC_DUTY_MAX ((1 << 16) - 1)

// Tiempo estimado (segundos) que tarda el servo continuo
// en moverse de la posición mínima (0) a la máxima (1)
#define SERVO_FULL_TRAVEL_TIME 2.0f

void servo_init(Servo *servo, int pin, float pos_min, float pos_max, servo_mode_t mode) {
    servo->pin = pin;
    servo->pos_min = pos_min;
    servo->pos_max = pos_max;
    servo->current_pos = pos_min;
    servo->timer = LEDC_TIMER_0;
    servo->channel = LEDC_CHANNEL_0;
    servo->mode = mode;

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = servo->timer,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = servo->channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = servo->timer,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    if (mode == SERVO_MODE_STANDARD) {
        servo_move_to(servo, pos_min);
    } else {
        // En continuo, arrancamos parado
        servo_set_speed(servo, 0.0f);
    }
}

void servo_write_pwm(Servo *servo, float position) {
    float normalized = (position - servo->pos_min) / (servo->pos_max - servo->pos_min);
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    float pulse_us = SERVO_MIN_PULSEWIDTH_US + normalized * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US);
    uint32_t duty = (uint32_t)((pulse_us / 20000.0f) * LEDC_DUTY_MAX);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, servo->channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, servo->channel);

    printf("[SERVO] Pin %d | Pos %.2f | Pulse %.0fus\n", servo->pin, position, pulse_us);
}

void servo_move_to(Servo *servo, float position) {
    if (position < servo->pos_min) position = servo->pos_min;
    if (position > servo->pos_max) position = servo->pos_max;
    servo->current_pos = position;
    servo_write_pwm(servo, position);
}

void servo_set_speed(Servo *servo, float speed) {
    if (servo->mode == SERVO_MODE_STANDARD) {
        float center = (servo->pos_min + servo->pos_max) / 2.0f;
        float span = (servo->pos_max - servo->pos_min) / 10.0f;
        float position = center + speed * span;
        servo_move_to(servo, position);
        return;
    }

    // Continuo: velocidad -1..1 → PWM
    float pulse_us = 1500.0f + speed * 500.0f;
    uint32_t duty = (uint32_t)((pulse_us / 20000.0f) * LEDC_DUTY_MAX);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, servo->channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, servo->channel);
}

void servo_set_normalized(Servo *servo, float target) {
    if (target < 0.0f) target = 0.0f;
    if (target > 1.0f) target = 1.0f;

    if (servo->mode == SERVO_MODE_STANDARD) {
        float position = servo->pos_min + target * (servo->pos_max - servo->pos_min);
        servo_move_to(servo, position);
        return;
    }

    // Servo continuo → simular posición con tiempo
    float delta = target - servo->current_pos;
    if (delta == 0) return;

    float direction = (delta > 0) ? 1.0f : -1.0f;
    float travel_time = fabs(delta) * SERVO_FULL_TRAVEL_TIME;

    printf("[SERVO360] Moving %.2f -> %.2f (%.2fs)\n", servo->current_pos, target, travel_time);

    servo_set_speed(servo, direction);
    vTaskDelay(pdMS_TO_TICKS(travel_time * 1000));
    servo_set_speed(servo, 0);

    servo->current_pos = target;
}
