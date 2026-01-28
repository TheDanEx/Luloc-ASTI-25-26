/*
 * Encoder Reader Component - Reads encoder pulses using PCNT and MCPWM
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "encoder_read.h"

static const char *TAG = "encoder_read";

#define ENC_A_GPIO 32
#define ENC_B_GPIO 33

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_H_LIM  32767
#define PCNT_L_LIM -32768

// Boot time reference (microseconds)
static int64_t boot_time_us = 0;
static volatile int16_t encoder_count = 0;
static volatile int16_t encoder_delta = 0;

/**
 * @brief Initialize PCNT encoder
 */
static void pcnt_encoder_init(void)
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENC_A_GPIO,
        .ctrl_gpio_num  = ENC_B_GPIO,
        .channel        = PCNT_CHANNEL_0,
        .unit           = PCNT_UNIT,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DEC,
        .lctrl_mode     = PCNT_MODE_REVERSE,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .counter_h_lim  = PCNT_H_LIM,
        .counter_l_lim  = PCNT_L_LIM,
    };

    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(PCNT_UNIT, 1000);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

/**
 * @brief Initialize encoder
 */
esp_err_t encoder_init(void)
{
    if (boot_time_us != 0) {
        ESP_LOGW(TAG, "Encoder already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing encoder...");
    
    gpio_set_pull_mode(ENC_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENC_B_GPIO, GPIO_PULLUP_ONLY);

    pcnt_encoder_init();

    // Get current time as reference point
    boot_time_us = esp_timer_get_time();
    encoder_count = 0;
    encoder_delta = 0;
    
    ESP_LOGI(TAG, "Encoder initialized successfully");
    return ESP_OK;
}

/**
 * @brief Update encoder readings
 */
esp_err_t encoder_update(void)
{
    int16_t current_count = 0;
    
    pcnt_get_counter_value(PCNT_UNIT, &current_count);
    encoder_delta = current_count - encoder_count;
    encoder_count = current_count;
    
    return ESP_OK;
}

/**
 * @brief Get current encoder reading
 */
esp_err_t encoder_read(encoder_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return ESP_FAIL;
    }

    encoder_update();

    // Read current encoder values
    data->count = encoder_count;
    data->delta = encoder_delta;
    data->elapsed_us = esp_timer_get_time() - boot_time_us;

    ESP_LOGD(TAG, "Count: %d | Delta: %d | Elapsed: %lld us", 
             data->count, data->delta, data->elapsed_us);

    return ESP_OK;
}

/**
 * @brief Get encoder count
 */
int16_t encoder_get_count(void)
{
    encoder_update();
    return encoder_count;
}

/**
 * @brief Get encoder delta (change since last read)
 */
int16_t encoder_get_delta(void)
{
    encoder_update();
    return encoder_delta;
}

/**
 * @brief Get encoder period in microseconds
 */
int64_t encoder_get_elapsed_us(void)
{
    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return 0;
    }
    return esp_timer_get_time() - boot_time_us;
}

/**
 * @brief Get elapsed time in milliseconds
 */
uint32_t encoder_get_elapsed_ms(void)
{
    return (uint32_t)(encoder_get_elapsed_us() / 1000);
}

/**
 * @brief Get elapsed time in seconds
 */
uint32_t encoder_get_elapsed_sec(void)
{
    return (uint32_t)(encoder_get_elapsed_us() / 1000000);
}

/**
 * @brief Clear encoder count
 */
void encoder_clear_count(void)
{
    pcnt_counter_clear(PCNT_UNIT);
    encoder_count = 0;
    encoder_delta = 0;
}
