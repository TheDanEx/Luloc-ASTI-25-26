/*
 * Encoder Sensor Component
 * Reads quadrature encoders using PCNT (Distance/Odometry) and MCPWM Capture (Speed/PID)
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include "encoder_sensor.h"
#include <malloc.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include <math.h>

static const char *TAG = "encoder_sensor";

/* =============================================================================
 * CONTEXT STRUCTURE
 * ========================================================================== */

typedef struct {
    encoder_sensor_config_t config;
    
    // PCNT Hardware (Distance & Speed Source)
    pcnt_unit_handle_t  pcnt_unit;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;
    
    // Distance State
    int64_t accumulated_distance_counts;
    int     last_hardware_pcnt_value;
    
    // Speed State
    int64_t last_speed_distance_counts;
    int64_t last_speed_time_us;
    
    bool is_initialized;
} encoder_sensor_context_t;

// =============================================================================
// Internal Helpers
// =============================================================================

/**
 * Sync software accumulator with hardware counter.
 * Reads the 16-bit PCNT hardware register and adds the delta to a 64-bit 
 * software counter to prevent overflow and handle direction switching.
 */
static void update_distance_accumulator(encoder_sensor_context_t *ctx)
{
    int current_hardware_value = 0;
    pcnt_unit_get_count(ctx->pcnt_unit, &current_hardware_value);
    
    int delta = current_hardware_value - ctx->last_hardware_pcnt_value;
    
    if (ctx->config.reverse_direction) {
        delta = -delta;
    }

    ctx->accumulated_distance_counts += delta;
    ctx->last_hardware_pcnt_value = current_hardware_value;
}

// =============================================================================
// Public API: Lifecycle
// =============================================================================

/**
 * Initialize a new quadrature encoder instance using the ESP32 Pulse Counter (PCNT).
 * This configuration uses X4 decoding (counting both edges of both A and B phases).
 */
encoder_sensor_handle_t encoder_sensor_init(const encoder_sensor_config_t *config)
{
    if (config == NULL) return NULL;

    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)calloc(1, sizeof(encoder_sensor_context_t));
    if (ctx == NULL) return NULL;

    memcpy(&ctx->config, config, sizeof(encoder_sensor_config_t));
    
    ESP_LOGI(TAG, "Initializing PCNT Encoder: Pins A=%d, B=%d, PPR=%d", 
             config->pin_a, config->pin_b, config->ppr);

    // GPIO Config: High-impedance inputs for the encoder signals
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_a) | (1ULL << config->pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Initializate PCNT unit
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &ctx->pcnt_unit));

    pcnt_chan_config_t chan_a_config = {.edge_gpio_num = config->pin_a, .level_gpio_num = config->pin_b};
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->pcnt_unit, &chan_a_config, &ctx->pcnt_chan_a));
    
    pcnt_chan_config_t chan_b_config = {.edge_gpio_num = config->pin_b, .level_gpio_num = config->pin_a};
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->pcnt_unit, &chan_b_config, &ctx->pcnt_chan_b));

    // Configure Quadrature X4 decoding actions
    pcnt_channel_set_edge_action(ctx->pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ctx->pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ctx->pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ctx->pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(ctx->pcnt_unit);
    pcnt_unit_clear_count(ctx->pcnt_unit);
    pcnt_unit_start(ctx->pcnt_unit);

    ctx->last_speed_time_us = esp_timer_get_time();
    ctx->last_speed_distance_counts = 0;

    ctx->is_initialized = true;
    return (encoder_sensor_handle_t)ctx;
}

/**
 * Clean up hardware resources and free memory.
 */
esp_err_t encoder_sensor_deinit(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;

    pcnt_unit_stop(ctx->pcnt_unit);
    pcnt_unit_disable(ctx->pcnt_unit);
    pcnt_del_channel(ctx->pcnt_chan_a);
    pcnt_del_channel(ctx->pcnt_chan_b);
    pcnt_del_unit(ctx->pcnt_unit);

    free(ctx);
    return ESP_OK;
}

// =============================================================================
// Public API: Metrics
// =============================================================================

/**
 * Get total distance traveled in meters.
 * Performs physical conversion using wheel diameter and gear ratio.
 */
float encoder_sensor_get_distance(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return 0.0f;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    update_distance_accumulator(ctx);
    
    double counts_per_motor_revolution = (double)(ctx->config.ppr * 4);
    double motor_revolutions = (double)ctx->accumulated_distance_counts / counts_per_motor_revolution;
    
    double wheel_revolutions = motor_revolutions;
    if (ctx->config.gear_ratio > 0.0f) {
        wheel_revolutions /= ctx->config.gear_ratio;
    }

    return (float)(wheel_revolutions * M_PI * ctx->config.wheel_diameter_m);
}

/**
 * Reset horizontal travel counter.
 */
esp_err_t encoder_sensor_reset_distance(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    update_distance_accumulator(ctx);
    ctx->accumulated_distance_counts = 0;
    
    // Prevent speed spike on next calculation
    ctx->last_speed_distance_counts = 0;
    ctx->last_speed_time_us = esp_timer_get_time();
    
    return ESP_OK;
}

/**
 * Get current angular velocity converted to linear speed (m/s).
 * Uses a time-delta approach for precision.
 */
float encoder_sensor_get_speed(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return 0.0f;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    update_distance_accumulator(ctx);
    int64_t current_time_us = esp_timer_get_time();
    int64_t current_counts = ctx->accumulated_distance_counts;
    
    int64_t delta_time_us = current_time_us - ctx->last_speed_time_us;
    
    if (delta_time_us <= 0) {
        return 0.0f;
    }
    
    int64_t delta_counts = current_counts - ctx->last_speed_distance_counts;
    
    ctx->last_speed_time_us = current_time_us;
    ctx->last_speed_distance_counts = current_counts;
    
    double elapsed_time_seconds = (double)delta_time_us / 1000000.0;
    
    // Convert pulses to distance
    double counts_per_motor_revolution = (double)(ctx->config.ppr * 4);
    double motor_revolutions = (double)delta_counts / counts_per_motor_revolution;
    
    double wheel_revolutions = motor_revolutions;
    if (ctx->config.gear_ratio > 0.0f) {
        wheel_revolutions /= ctx->config.gear_ratio;
    }

    double distance_meters = wheel_revolutions * M_PI * ctx->config.wheel_diameter_m;
    
    // V = d / t
    return (float)(distance_meters / elapsed_time_seconds);
}
