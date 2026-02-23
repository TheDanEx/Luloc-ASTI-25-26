#include "servo.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_MID_US 1500
#define SERVO_FREQ   50

// Cuánto tarda el servo en moverse de 0 a 1 (en segundos)
#define SERVO_FULL_TRAVEL_TIME 2.0f

void servo_init(Servo *servo, int pin, float pos_min, float pos_max, servo_mode_t mode) {
    servo->pin = pin;
    servo->pos_min = pos_min;
    servo->pos_max = pos_max;
    servo->current_pos = 0.0f;
    servo->mode = mode;
    servo->channel = LEDC_CHANNEL_0;
    servo->timer = LEDC_TIMER_0;

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = servo->timer,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = servo->channel,
        .timer_sel = servo->timer,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
}

void servo_write_pwm(Servo *servo, float us) {
    float duty = (us / 20000.0f) * ((1 << 16) - 1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo->channel, (uint32_t)duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo->channel);
}

void servo_set_speed(Servo *servo, float speed) {
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    float us = SERVO_MID_US + speed * (SERVO_MAX_US - SERVO_MID_US);
    servo_write_pwm(servo, us);
}

void servo_set_normalized(Servo *servo, float target_pos) {
    if (servo->mode != SERVO_MODE_CONTINUOUS) return;
    if (target_pos > 1.0f) target_pos = 1.0f;
    if (target_pos < 0.0f) target_pos = 0.0f;

    float delta = target_pos - servo->current_pos;
    if (delta == 0) return;

    float direction = (delta > 0) ? 1.0f : -1.0f;
    float travel_time = SERVO_FULL_TRAVEL_TIME * fabs(delta);

    servo_set_speed(servo, direction);
    vTaskDelay(pdMS_TO_TICKS(travel_time * 1000));
    servo_set_speed(servo, 0);

    servo->current_pos = target_pos;
}
