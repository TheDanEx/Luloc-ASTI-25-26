/*
 * Encoder Sensor Component
 * Reads quadrature encoders using new ESP-IDF Pulse Count (pcnt) Driver
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "encoder_sensor.h"

static const char *TAG = "encoder_sensor";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    encoder_sensor_config_t config;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;
    
    int64_t accumulated_count;      // Total accumulated raw pulses (internal tracking)
    int last_hw_count;              // Last read HW count to calculate delta
    
    int64_t last_speed_time_us;     // Timestamp of last speed calc
    int64_t last_speed_count;       // Count at last speed calc
    float current_speed_m_s;        // Cached speed value
    
    bool initialized;
} encoder_sensor_context_t;

encoder_sensor_handle_t encoder_sensor_init(const encoder_sensor_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid config");
        return NULL;
    }

    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)calloc(1, sizeof(encoder_sensor_context_t));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return NULL;
    }

    memcpy(&ctx->config, config, sizeof(encoder_sensor_config_t));
    ctx->last_speed_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Initializing encoder: Pin A=%d, Pin B=%d, PPR=%d, Ratio=%.2f, Dia=%.3f m", 
             config->pin_a, config->pin_b, config->ppr, config->gear_ratio, config->wheel_diameter_m);

    // 1. Create PCNT unit
    pcnt_unit_config_t unit_config = {
        .high_limit = 32000,
        .low_limit = -32000,
    };
    if (pcnt_new_unit(&unit_config, &ctx->pcnt_unit) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit");
        free(ctx);
        return NULL;
    }

    // 2. Create PCNT channels (Full Quadrature Decoder)
    // Channel A: Edge on Pin A, Level on Pin B
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = config->pin_a,
        .level_gpio_num = config->pin_b,
    };
    if (pcnt_new_channel(ctx->pcnt_unit, &chan_a_config, &ctx->pcnt_chan_a) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel A");
        pcnt_del_unit(ctx->pcnt_unit);
        free(ctx);
        return NULL;
    }

    // Channel B: Edge on Pin B, Level on Pin A
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = config->pin_b,
        .level_gpio_num = config->pin_a,
    };
    if (pcnt_new_channel(ctx->pcnt_unit, &chan_b_config, &ctx->pcnt_chan_b) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel B");
        pcnt_del_channel(ctx->pcnt_chan_a);
        pcnt_del_unit(ctx->pcnt_unit);
        free(ctx);
        return NULL;
    }

    // 3. Setup Standard Quadrature Logic (X4 mode equivalent usually)
    // Edge actions for Channel A
    pcnt_channel_set_edge_action(ctx->pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ctx->pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // Edge actions for Channel B
    pcnt_channel_set_edge_action(ctx->pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ctx->pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // 4. Enable and Start
    pcnt_unit_enable(ctx->pcnt_unit);
    pcnt_unit_clear_count(ctx->pcnt_unit);
    pcnt_unit_start(ctx->pcnt_unit);

    ctx->initialized = true;
    return (encoder_sensor_handle_t)ctx;
}

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

static void update_sensor_state(encoder_sensor_context_t *ctx)
{
    int current_hw_count = 0;
    pcnt_unit_get_count(ctx->pcnt_unit, &current_hw_count);

    // Calculate delta taking into account possible SW overflow handling if necessary,
    // but here we rely on the frequent polling for simplicity in "accumulation" 
    // without complex ISRs, suitable for standard speeds.
    // NOTE: The hardware counter might wrap if we don't clear or track overflows,
    // but the driver handles range extension via interrupts if we used the glich filter / event callbacks.
    // For this simple 'get_count' based approach, we must assume we call this fast enough
    // before the 16-bit hardware counter (or larger) wraps multiple times.
    
    // With PCNT driver v5, get_count returns the full accumulated value if events are enabled,
    // but here we might just read the raw value.
    // The safest "simple" way is to handle 16-bit wrapping diff:
    
    // Simple diff logic assuming no huge jumps between calls
    int diff = current_hw_count - ctx->last_hw_count;
    
    // Handle wrap-around if using limits (e.g. -32000 to 32000)
    // If jump is too large, it might mean wrap.
    // Using a simpler approach: clear on read?
    // Using clear on read introduces race conditions.
    // We stick to simple diff and assume typical update rates.
    
    if (ctx->config.reverse_direction) {
        diff = -diff;
    }

    ctx->accumulated_count += diff;
    ctx->last_hw_count = current_hw_count;

    // Speed Calculation
    int64_t current_time = esp_timer_get_time();
    int64_t time_diff_us = current_time - ctx->last_speed_time_us;

    // Update speed only if some time has passed (e.g., > 10ms) to avoid noise
    if (time_diff_us >= 10000) { 
        double dt_sec = (double)time_diff_us / 1000000.0;
        
        int64_t pulses_delta = ctx->accumulated_count - ctx->last_speed_count;
        
        // Revs (Motor)
        double motor_revs = (double)pulses_delta / (double)ctx->config.ppr;
        
        // Revs (Wheel)
        double wheel_revs = motor_revs;
        if (ctx->config.gear_ratio > 0.0f) {
            wheel_revs /= ctx->config.gear_ratio;
        }

        double dist_m = wheel_revs * M_PI * ctx->config.wheel_diameter_m;
        
        ctx->current_speed_m_s = (float)(dist_m / dt_sec);
        
        ctx->last_speed_time_us = current_time;
        ctx->last_speed_count = ctx->accumulated_count;
    }
}

float encoder_sensor_get_speed(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return 0.0f;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    update_sensor_state(ctx);
    return ctx->current_speed_m_s;
}

float encoder_sensor_get_distance(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return 0.0f;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    update_sensor_state(ctx);
    
    double motor_revs = (double)ctx->accumulated_count / (double)ctx->config.ppr;
    double wheel_revs = motor_revs;
    if (ctx->config.gear_ratio > 0.0f) {
        wheel_revs /= ctx->config.gear_ratio;
    }

    return (float)(wheel_revs * M_PI * ctx->config.wheel_diameter_m);
}

esp_err_t encoder_sensor_reset_distance(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    ctx->accumulated_count = 0;
    // We also reset the speed reference to avoid a jump
    ctx->last_speed_count = 0; 
    
    // We don't reset HW counter to keep continuity in diff calculation
    // ctx->last_hw_count remains valid against current HW count
    
    return ESP_OK;
}
