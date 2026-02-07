/*
 * Simple Servo driver (LEDC) - Implementation
 * Based on user-provided example, adapted into component file.
 * SPDX-License-Identifier: MIT
 */

/*
 * Servo component implementation (handle-based)
 */

#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "servo";

typedef struct {
    servo_config_t cfg;
    uint32_t max_duty;
} servo_ctx_t;

servo_handle_t servo_init(const servo_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "servo_init: NULL config");
        return NULL;
    }

    servo_ctx_t *ctx = calloc(1, sizeof(servo_ctx_t));
    if (!ctx) {
        ESP_LOGE(TAG, "servo_init: malloc failed");
        return NULL;
    }

    ctx->cfg = *config;

    ledc_timer_config_t timer_conf = {
        .speed_mode = ctx->cfg.speed_mode,
        .timer_num = ctx->cfg.timer_num,
        .duty_resolution = ctx->cfg.duty_resolution,
        .freq_hz = ctx->cfg.freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", err);
        free(ctx);
        return NULL;
    }

    ledc_channel_config_t ch_conf = {
        .speed_mode = ctx->cfg.speed_mode,
        .channel = ctx->cfg.channel,
        .timer_sel = ctx->cfg.timer_num,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ctx->cfg.gpio_num,
        .duty = 0,
        .hpoint = 0,
    };

    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %d", err);
        free(ctx);
        return NULL;
    }

    /* Precompute max duty */
    ctx->max_duty = ((uint32_t)1 << ctx->cfg.duty_resolution) - 1;

    /* If config provided bounds use them, otherwise keep 500..2500 defaults */
    if (ctx->cfg.min_pulse_us == 0) ctx->cfg.min_pulse_us = 500;
    if (ctx->cfg.max_pulse_us == 0) ctx->cfg.max_pulse_us = 2500;

    ESP_LOGI(TAG, "Servo init GPIO=%d timer=%d channel=%d freq=%u res=%d bounds=%uus-%uus",
             ctx->cfg.gpio_num, ctx->cfg.timer_num, ctx->cfg.channel,
             ctx->cfg.freq_hz, ctx->cfg.duty_resolution,
             ctx->cfg.min_pulse_us, ctx->cfg.max_pulse_us);

    return (servo_handle_t)ctx;
}

esp_err_t servo_deinit(servo_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    servo_ctx_t *ctx = (servo_ctx_t *)handle;

    /* Stop PWM on channel (set duty 0 and disable) */
    ledc_set_duty(ctx->cfg.speed_mode, ctx->cfg.channel, 0);
    ledc_update_duty(ctx->cfg.speed_mode, ctx->cfg.channel);
    ledc_stop(ctx->cfg.speed_mode, ctx->cfg.channel, 0);

    free(ctx);
    return ESP_OK;
}

esp_err_t servo_set_bounds(servo_handle_t handle, uint32_t min_us, uint32_t max_us)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    if (min_us == 0 || max_us <= min_us) return ESP_ERR_INVALID_ARG;
    servo_ctx_t *ctx = (servo_ctx_t *)handle;
    ctx->cfg.min_pulse_us = min_us;
    ctx->cfg.max_pulse_us = max_us;
    return ESP_OK;
}

static esp_err_t servo_write_pulse(servo_ctx_t *ctx, uint32_t pulse_us)
{
    if (ctx == NULL) return ESP_ERR_INVALID_ARG;

    uint32_t period_us = 1000000UL / ctx->cfg.freq_hz;
    if (period_us == 0) return ESP_ERR_INVALID_ARG;

    uint64_t duty = ((uint64_t)pulse_us * ctx->max_duty) / period_us;
    if (duty > ctx->max_duty) duty = ctx->max_duty;

    esp_err_t err = ledc_set_duty(ctx->cfg.speed_mode, ctx->cfg.channel, (uint32_t)duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(ctx->cfg.speed_mode, ctx->cfg.channel);
}

esp_err_t servo_move_normalized(servo_handle_t handle, float position)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    if (position < 0.0f) position = 0.0f;
    if (position > 1.0f) position = 1.0f;
    servo_ctx_t *ctx = (servo_ctx_t *)handle;
    uint32_t target = ctx->cfg.min_pulse_us + (uint32_t)(position * (float)(ctx->cfg.max_pulse_us - ctx->cfg.min_pulse_us));
    return servo_write_pulse(ctx, target);
}

esp_err_t servo_move_us(servo_handle_t handle, uint32_t pulse_us)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    servo_ctx_t *ctx = (servo_ctx_t *)handle;
    if (pulse_us < ctx->cfg.min_pulse_us) pulse_us = ctx->cfg.min_pulse_us;
    if (pulse_us > ctx->cfg.max_pulse_us) pulse_us = ctx->cfg.max_pulse_us;
    return servo_write_pulse(ctx, pulse_us);
}

