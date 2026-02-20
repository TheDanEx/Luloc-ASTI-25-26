/*
 * Encoder Sensor Component
 * Reads quadrature encoders using PCNT (Distance/Odometry) and MCPWM Capture (Speed/PID)
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "encoder_sensor.h"

static const char *TAG = "encoder_sensor";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    encoder_sensor_config_t config;
    
    // PCNT para Distancia (Odometría)
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;
    int64_t accumulated_count;      
    int last_hw_count;              
    
    // MCPWM Capture para Velocidad instantánea
    mcpwm_cap_timer_handle_t cap_timer;
    mcpwm_cap_channel_handle_t cap_chan;
    uint32_t last_cap_ticks;            // Ticks de la captura anterior
    volatile uint32_t latest_pulse_dt;  // Delta de tiempo entre los dos últimos flancos
    volatile int latest_dir;            // Dirección instantánea (1 o -1)
    volatile int64_t last_pulse_time_us;// Marca de tiempo para detectar si el motor se detiene
    
    bool initialized;
} encoder_sensor_context_t;


// --- INTERRUPCIÓN MCPWM (Solo para medir velocidad) ---
static bool IRAM_ATTR mcpwm_cap_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) 
{
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)user_data;
    
    uint32_t current_ticks = edata->cap_value;
    ctx->latest_pulse_dt = current_ticks - ctx->last_cap_ticks;
    ctx->last_cap_ticks = current_ticks;
    
    // Leer el pin B para saber la dirección (A sube, si B está alto = reverse, si está bajo = forward)
    int dir = gpio_get_level(ctx->config.pin_b) ? -1 : 1;
    if (ctx->config.reverse_direction) {
        dir = -dir;
    }
    ctx->latest_dir = dir;
    ctx->last_pulse_time_us = esp_timer_get_time();

    return false; // No necesitamos despertar ninguna tarea
}


// --- INICIALIZACIÓN ---
encoder_sensor_handle_t encoder_sensor_init(const encoder_sensor_config_t *config)
{
    if (config == NULL) return NULL;

    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)calloc(1, sizeof(encoder_sensor_context_t));
    if (ctx == NULL) return NULL;

    memcpy(&ctx->config, config, sizeof(encoder_sensor_config_t));
    ctx->last_pulse_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Initializing Odometry & Speed encoder: Pins A=%d, B=%d", config->pin_a, config->pin_b);

    // 0. Configurar GPIOs (Sin Pull-ups, usando los del hardware)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_a) | (1ULL << config->pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // ==========================================
    // 1. CONFIGURACIÓN PCNT (DISTANCIA X4)
    // ==========================================
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    pcnt_new_unit(&unit_config, &ctx->pcnt_unit);

    pcnt_chan_config_t chan_a_config = {.edge_gpio_num = config->pin_a, .level_gpio_num = config->pin_b};
    pcnt_new_channel(ctx->pcnt_unit, &chan_a_config, &ctx->pcnt_chan_a);
    pcnt_chan_config_t chan_b_config = {.edge_gpio_num = config->pin_b, .level_gpio_num = config->pin_a};
    pcnt_new_channel(ctx->pcnt_unit, &chan_b_config, &ctx->pcnt_chan_b);

    pcnt_channel_set_edge_action(ctx->pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ctx->pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ctx->pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ctx->pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(ctx->pcnt_unit);
    pcnt_unit_clear_count(ctx->pcnt_unit);
    pcnt_unit_start(ctx->pcnt_unit);

    // ==========================================
    // 2. CONFIGURACIÓN MCPWM (VELOCIDAD X1)
    // ==========================================
    mcpwm_capture_timer_config_t cap_timer_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 1, // Automático
        .resolution_hz = 1000000, // 1 MHz = 1 tick por microsegundo
    };
    mcpwm_new_capture_timer(&cap_timer_conf, &ctx->cap_timer);

    mcpwm_capture_channel_config_t cap_chan_conf = {
        .gpio_num = config->pin_a, // Solo medimos el pin A
        .prescale = 1,
        .flags.pos_edge = true,    // Solo flanco de subida (X1, evita asimetría magnética)
        .flags.neg_edge = false,
        .flags.pull_up = false,
        .flags.pull_down = false,
    };
    mcpwm_new_capture_channel(ctx->cap_timer, &cap_chan_conf, &ctx->cap_chan);

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = mcpwm_cap_callback,
    };
    mcpwm_capture_channel_register_event_callbacks(ctx->cap_chan, &cbs, ctx);
    
    mcpwm_capture_channel_enable(ctx->cap_chan);
    mcpwm_capture_timer_enable(ctx->cap_timer);
    mcpwm_capture_timer_start(ctx->cap_timer);

    ctx->initialized = true;
    return (encoder_sensor_handle_t)ctx;
}


// --- ACTUALIZAR SOLO DISTANCIA ---
static void update_distance_state(encoder_sensor_context_t *ctx)
{
    int current_hw_count = 0;
    pcnt_unit_get_count(ctx->pcnt_unit, &current_hw_count);

    // Manejo de desbordamiento de 16-bit
    int16_t diff = (int16_t)(current_hw_count - ctx->last_hw_count);
    
    if (ctx->config.reverse_direction) {
        diff = -diff;
    }

    ctx->accumulated_count += diff;
    ctx->last_hw_count = current_hw_count;
}


// --- FUNCIONES PÚBLICAS ---

float encoder_sensor_get_distance(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return 0.0f;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    update_distance_state(ctx);
    
    // Distancia usa resolución X4 (4 cuentas por pulso base)
    double counts_per_motor_rev = (double)(ctx->config.ppr * 4);
    double motor_revs = (double)ctx->accumulated_count / counts_per_motor_rev;
    
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
    
    // Actualizamos la lectura de hardware actual antes de poner a 0
    // para no perder pulsos no registrados en la diferencia
    int current_hw_count = 0;
    pcnt_unit_get_count(ctx->pcnt_unit, &current_hw_count);
    ctx->last_hw_count = current_hw_count;
    
    ctx->accumulated_count = 0;
    return ESP_OK;
}

float encoder_sensor_get_speed(encoder_sensor_handle_t handle)
{
    if (handle == NULL) return 0.0f;
    encoder_sensor_context_t *ctx = (encoder_sensor_context_t *)handle;
    
    int64_t now = esp_timer_get_time();
    
    // Si han pasado más de 100ms (0.1s) sin ningún pulso, el motor está parado
    if (now - ctx->last_pulse_time_us > 100000) {
        return 0.0f;
    }
    
    // Proteger las variables volátiles
    uint32_t dt_us = ctx->latest_pulse_dt;
    int dir = ctx->latest_dir;
    
    if (dt_us == 0) return 0.0f; // Evitar división por 0
    
    double time_s = (double)dt_us / 1000000.0;
    
    // Velocidad usa resolución X1 (1 cuenta por pulso base porque medimos periodo)
    double motor_revs_per_pulse = 1.0 / (double)ctx->config.ppr;
    double wheel_revs_per_pulse = motor_revs_per_pulse;
    
    if (ctx->config.gear_ratio > 0.0f) {
        wheel_revs_per_pulse /= ctx->config.gear_ratio;
    }
    
    double dist_m_per_pulse = wheel_revs_per_pulse * M_PI * ctx->config.wheel_diameter_m;
    
    return (float)(dir * (dist_m_per_pulse / time_s));
}