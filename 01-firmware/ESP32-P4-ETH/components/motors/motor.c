/*
 * Motor Class - Controls motors with encoder feedback using PCNT
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "motor.h"

static const char *TAG = "motor";

/**
 * @brief Internal motor context structure
 */
typedef struct {
    motor_config_t config;          // Motor configuration
    int32_t current_count;          // Current encoder count
    int32_t last_count;             // Last encoder count (for delta calculation)
    int64_t last_speed_time_us;     // Timestamp of last speed calculation
    float last_speed_rpm;           // Cached speed value
    pcnt_unit_t pcnt_unit;          // PCNT unit handle
    bool initialized;               // Initialization flag
} motor_context_t;

/**
 * @brief PCNT overflow handler for the motor encoder
 */
static void motor_pcnt_overflow_handler(void *arg)
{
    motor_context_t *motor = (motor_context_t *)arg;
    if (motor == NULL) {
        return;
    }
    
    ESP_LOGD(TAG, "PCNT overflow detected on motor");
}

/**
 * @brief Initialize motor with encoder pins and PCNT unit
 */
motor_handle_t motor_init(const motor_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid config pointer");
        return NULL;
    }

    if (config->encoder_pin_a < 0 || config->encoder_pin_b < 0) {
        ESP_LOGE(TAG, "Invalid encoder pins: A=%d, B=%d", 
                 config->encoder_pin_a, config->encoder_pin_b);
        return NULL;
    }

    if (config->ppr <= 0) {
        ESP_LOGE(TAG, "Invalid PPR value: %d", config->ppr);
        return NULL;
    }

    // Allocate motor context
    motor_context_t *motor = (motor_context_t *)malloc(sizeof(motor_context_t));
    if (motor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate motor context");
        return NULL;
    }

    // Copy configuration
    memcpy(&motor->config, config, sizeof(motor_config_t));
    motor->current_count = 0;
    motor->last_count = 0;
    motor->last_speed_rpm = 0.0f;
    motor->last_speed_time_us = esp_timer_get_time();
    motor->pcnt_unit = config->pcnt_unit;
    motor->initialized = false;

    // Configure GPIO pins for encoder input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->encoder_pin_a) | (1ULL << config->encoder_pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        free(motor);
        return NULL;
    }

    ESP_LOGD(TAG, "GPIO pins configured: A=%d, B=%d", 
             config->encoder_pin_a, config->encoder_pin_b);

    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = config->encoder_pin_a,
        .ctrl_gpio_num = config->encoder_pin_b,
        .unit = config->pcnt_unit,
        .channel = PCNT_CHANNEL_0,
        .pos_mode = PCNT_COUNT_INC,      // Count up on positive edge
        .neg_mode = PCNT_COUNT_DIS,      // Don't count on negative edge
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting when control is high
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep counting when control is low
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };

    ret = pcnt_unit_config(&pcnt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT unit config failed: %s", esp_err_to_name(ret));
        free(motor);
        return NULL;
    }

    // Enable PCNT input filter for noise immunity
    pcnt_set_filter_value(config->pcnt_unit, 10);
    
    // Register overflow interrupt handler
    pcnt_isr_register(motor_pcnt_overflow_handler, motor, 0, NULL);

    // Enable and reset the PCNT unit
    ret = pcnt_counter_pause(config->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT pause failed: %s", esp_err_to_name(ret));
        free(motor);
        return NULL;
    }

    ret = pcnt_counter_clear(config->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT clear failed: %s", esp_err_to_name(ret));
        free(motor);
        return NULL;
    }

    ret = pcnt_counter_resume(config->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT resume failed: %s", esp_err_to_name(ret));
        free(motor);
        return NULL;
    }

    motor->initialized = true;

    ESP_LOGI(TAG, "Motor initialized successfully - PCNT Unit: %d, PPR: %d", 
             config->pcnt_unit, config->ppr);

    return (motor_handle_t)motor;
}

/**
 * @brief Deinitialize motor and cleanup resources
 */
esp_err_t motor_deinit(motor_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    if (!motor->initialized) {
        ESP_LOGW(TAG, "Motor not initialized");
        free(motor);
        return ESP_OK;
    }

    // Pause PCNT unit
    pcnt_counter_pause(motor->pcnt_unit);

    // Free the context
    free(motor);

    ESP_LOGI(TAG, "Motor deinitialized");

    return ESP_OK;
}

/**
 * @brief Read current encoder count
 */
int32_t motor_get_encoder_count(motor_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid motor handle");
        return 0;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    if (!motor->initialized) {
        ESP_LOGW(TAG, "Motor not initialized");
        return 0;
    }

    int16_t count = 0;
    esp_err_t ret = pcnt_get_counter_value(motor->pcnt_unit, &count);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PCNT counter: %s", esp_err_to_name(ret));
        return motor->current_count;
    }

    motor->current_count = (int32_t)count;
    return motor->current_count;
}

/**
 * @brief Reset encoder count to zero
 */
esp_err_t motor_reset_encoder(motor_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    if (!motor->initialized) {
        ESP_LOGW(TAG, "Motor not initialized");
        return ESP_FAIL;
    }

    esp_err_t ret = pcnt_counter_clear(motor->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT counter: %s", esp_err_to_name(ret));
        return ret;
    }

    motor->current_count = 0;
    motor->last_count = 0;

    ESP_LOGD(TAG, "Encoder count reset");

    return ESP_OK;
}

/**
 * @brief Get encoder count delta since last call
 */
int32_t motor_get_count_delta(motor_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    int32_t current = motor_get_encoder_count(handle);
    int32_t delta = current - motor->last_count;
    motor->last_count = current;

    return delta;
}

/**
 * @brief Calculate and get current motor speed in RPM
 */
float motor_get_speed_rpm(motor_handle_t handle)
{
    if (handle == NULL) {
        return 0.0f;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    if (!motor->initialized || motor->config.ppr == 0) {
        ESP_LOGW(TAG, "Motor not properly initialized or PPR is 0");
        return 0.0f;
    }

    int32_t delta = motor_get_count_delta(handle);
    int64_t current_time_us = esp_timer_get_time();
    int64_t time_delta_us = current_time_us - motor->last_speed_time_us;

    if (time_delta_us == 0) {
        return motor->last_speed_rpm;
    }

    motor->last_speed_time_us = current_time_us;

    // Speed calculation:
    // delta = encoder pulses in time_delta_us
    // revolutions = delta / PPR
    // time in minutes = time_delta_us / (1000000 * 60)
    // RPM = revolutions / time_in_minutes
    
    float time_in_minutes = (float)time_delta_us / (1000000.0f * 60.0f);
    float revolutions = (float)delta / (float)motor->config.ppr;
    
    if (time_in_minutes > 0) {
        motor->last_speed_rpm = revolutions / time_in_minutes;
    } else {
        motor->last_speed_rpm = 0.0f;
    }

    return motor->last_speed_rpm;
}

/**
 * @brief Get motor state including speed calculations
 */
esp_err_t motor_get_state(motor_handle_t handle, motor_state_t *state)
{
    if (handle == NULL || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    if (!motor->initialized) {
        ESP_LOGW(TAG, "Motor not initialized");
        return ESP_FAIL;
    }

    // Get encoder data
    state->encoder_count = motor_get_encoder_count(handle);
    state->count_delta = motor_get_count_delta(handle);
    state->speed_rpm = motor_get_speed_rpm(handle);
    
    // Calculate derived speed values
    state->speed_rps = state->speed_rpm / 60.0f;  // Revolutions per second
    state->speed_rad_s = state->speed_rps * 6.28318530718f;  // Angular velocity in rad/s (2π)
    
    state->timestamp_ms = (int32_t)(esp_timer_get_time() / 1000);

    ESP_LOGD(TAG, "Motor state - Count: %ld, Delta: %ld, RPM: %.2f, Rad/s: %.2f",
             state->encoder_count, state->count_delta, state->speed_rpm, state->speed_rad_s);

    return ESP_OK;
}

/**
 * @brief Generate JSON representation of motor state
 */
cJSON* motor_state_to_json(motor_handle_t handle)
{
    if (handle == NULL) {
        return NULL;
    }

    motor_state_t state;
    esp_err_t ret = motor_get_state(handle, &state);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get motor state");
        return NULL;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return NULL;
    }

    cJSON_AddNumberToObject(root, "encoder_count", state.encoder_count);
    cJSON_AddNumberToObject(root, "count_delta", state.count_delta);
    cJSON_AddNumberToObject(root, "speed_rpm", state.speed_rpm);
    cJSON_AddNumberToObject(root, "speed_rps", state.speed_rps);
    cJSON_AddNumberToObject(root, "speed_rad_s", state.speed_rad_s);
    cJSON_AddNumberToObject(root, "timestamp_ms", state.timestamp_ms);

    return root;
}

/**
 * @brief Get motor configuration
 */
esp_err_t motor_get_config(motor_handle_t handle, motor_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_context_t *motor = (motor_context_t *)handle;

    memcpy(config, &motor->config, sizeof(motor_config_t));

    return ESP_OK;
}
