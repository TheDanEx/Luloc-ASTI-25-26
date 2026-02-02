/*
 * Encoder Reader Component - Reads encoder pulses using PCNT and MCPWM
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pcnt.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "encoder_read.h"

static const char *TAG = "encoder_read";

/**
 * @brief Internal encoder context structure
 */
typedef struct {
    encoder_config_t config;        // Encoder configuration
    int32_t current_count;          // Current encoder count
    int32_t last_count;             // Last encoder count (for delta)
    int64_t last_speed_time_us;     // Timestamp of last speed calculation
    float last_speed_rpm;           // Cached speed value
    float last_speed_rps;           // Cached RPS value
    float last_speed_rad_s;         // Cached rad/s value
    bool initialized;               // Initialization flag
} encoder_context_t;

/**
 * @brief PCNT overflow handler for encoder
 */
static void encoder_overflow_handler(void *arg)
{
    encoder_context_t *encoder = (encoder_context_t *)arg;
    if (encoder != NULL) {
        ESP_LOGD(TAG, "PCNT overflow detected on encoder");
    }
}

/**
 * @brief Initialize encoder with configuration
 */
encoder_handle_t encoder_init(const encoder_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid config pointer");
        return NULL;
    }

    if (config->pin_a < 0 || config->pin_b < 0) {
        ESP_LOGE(TAG, "Invalid encoder pins: A=%d, B=%d", 
                 config->pin_a, config->pin_b);
        return NULL;
    }

    if (config->ppr <= 0) {
        ESP_LOGE(TAG, "Invalid PPR value: %d", config->ppr);
        return NULL;
    }

    // Allocate encoder context
    encoder_context_t *encoder = (encoder_context_t *)malloc(sizeof(encoder_context_t));
    if (encoder == NULL) {
        ESP_LOGE(TAG, "Failed to allocate encoder context");
        return NULL;
    }

    // Copy configuration
    memcpy(&encoder->config, config, sizeof(encoder_config_t));
    encoder->current_count = 0;
    encoder->last_count = 0;
    encoder->last_speed_rpm = 0.0f;
    encoder->last_speed_rps = 0.0f;
    encoder->last_speed_rad_s = 0.0f;
    encoder->last_speed_time_us = esp_timer_get_time();
    encoder->initialized = false;

    // Configure GPIO pins for encoder input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_a) | (1ULL << config->pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        free(encoder);
        return NULL;
    }

    ESP_LOGD(TAG, "GPIO pins configured: A=%d, B=%d", config->pin_a, config->pin_b);

    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = config->pin_a,
        .ctrl_gpio_num = config->pin_b,
        .unit = config->pcnt_unit,
        .channel = PCNT_CHANNEL_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };

    ret = pcnt_unit_config(&pcnt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT unit config failed: %s", esp_err_to_name(ret));
        free(encoder);
        return NULL;
    }

    // Enable input filter for noise immunity
    pcnt_set_filter_value(config->pcnt_unit, 10);
    
    // Register overflow interrupt handler
    pcnt_isr_register(encoder_overflow_handler, encoder, 0, NULL);

    // Initialize counter
    ret = pcnt_counter_pause(config->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT pause failed: %s", esp_err_to_name(ret));
        free(encoder);
        return NULL;
    }

    ret = pcnt_counter_clear(config->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT clear failed: %s", esp_err_to_name(ret));
        free(encoder);
        return NULL;
    }

    ret = pcnt_counter_resume(config->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCNT resume failed: %s", esp_err_to_name(ret));
        free(encoder);
        return NULL;
    }

    encoder->initialized = true;

    ESP_LOGI(TAG, "Encoder initialized - Unit: %d, PPR: %d, Pins: A=%d B=%d", 
             config->pcnt_unit, config->ppr, config->pin_a, config->pin_b);

    return (encoder_handle_t)encoder;
}

/**
 * @brief Deinitialize encoder
 */
esp_err_t encoder_deinit(encoder_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (encoder->initialized) {
        pcnt_counter_pause(encoder->config.pcnt_unit);
    }

    free(encoder);
    ESP_LOGI(TAG, "Encoder deinitialized");

    return ESP_OK;
}

/**
 * @brief Read current encoder count
 */
int32_t encoder_read_count(encoder_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid encoder handle");
        return 0;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (!encoder->initialized) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return 0;
    }

    int16_t count = 0;
    esp_err_t ret = pcnt_get_counter_value(encoder->config.pcnt_unit, &count);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PCNT counter: %s", esp_err_to_name(ret));
        return encoder->current_count;
    }

    encoder->current_count = (int32_t)count;
    return encoder->current_count;
}

/**
 * @brief Get encoder count delta since last read
 */
int32_t encoder_get_delta(encoder_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    int32_t current = encoder_read_count(handle);
    int32_t delta = current - encoder->last_count;
    encoder->last_count = current;

    return delta;
}

/**
 * @brief Internal function to calculate speed from encoder delta
 */
static void encoder_calculate_speed(encoder_context_t *encoder)
{
    if (!encoder->initialized || encoder->config.ppr == 0) {
        encoder->last_speed_rpm = 0.0f;
        encoder->last_speed_rps = 0.0f;
        encoder->last_speed_rad_s = 0.0f;
        return;
    }

    int32_t delta = encoder_read_count(encoder) - encoder->last_count;
    encoder->last_count = encoder_read_count(encoder);

    int64_t current_time_us = esp_timer_get_time();
    int64_t time_delta_us = current_time_us - encoder->last_speed_time_us;

    if (time_delta_us == 0) {
        return;
    }

    encoder->last_speed_time_us = current_time_us;

    // RPM = (delta / PPR) / (time_in_minutes)
    // time_in_minutes = time_delta_us / (1,000,000 * 60)
    float time_in_minutes = (float)time_delta_us / (1000000.0f * 60.0f);
    float revolutions = (float)delta / (float)encoder->config.ppr;
    
    if (time_in_minutes > 0) {
        encoder->last_speed_rpm = revolutions / time_in_minutes;
    } else {
        encoder->last_speed_rpm = 0.0f;
    }

    encoder->last_speed_rps = encoder->last_speed_rpm / 60.0f;
    encoder->last_speed_rad_s = encoder->last_speed_rps * 6.28318530718f;

    ESP_LOGD(TAG, "Speed: %.2f RPM (delta: %ld, time: %lld us)", 
             encoder->last_speed_rpm, delta, time_delta_us);
}

/**
 * @brief Get complete encoder state (primary interface)
 */
esp_err_t encoder_get_state(encoder_handle_t handle, encoder_state_t *state)
{
    if (handle == NULL || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (!encoder->initialized) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return ESP_FAIL;
    }

    // Get count and delta
    state->count = encoder_read_count(handle);
    state->delta = encoder_get_delta(handle);
    
    // Calculate and update speed values
    encoder_calculate_speed(encoder);
    state->speed_rpm = encoder->last_speed_rpm;
    state->speed_rps = encoder->last_speed_rps;
    state->speed_rad_s = encoder->last_speed_rad_s;
    
    state->timestamp_ms = (int32_t)(esp_timer_get_time() / 1000);

    ESP_LOGD(TAG, "State - Count: %ld, Delta: %ld, RPM: %.2f, Rad/s: %.2f",
             state->count, state->delta, state->speed_rpm, state->speed_rad_s);

    return ESP_OK;
}

/**
 * @brief Get cached speed in RPM
 */
float encoder_get_speed_rpm(encoder_handle_t handle)
{
    if (handle == NULL) {
        return 0.0f;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (!encoder->initialized) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return 0.0f;
    }

    return encoder->last_speed_rpm;
}

/**
 * @brief Get cached speed in revolutions per second
 */
float encoder_get_speed_rps(encoder_handle_t handle)
{
    if (handle == NULL) {
        return 0.0f;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (!encoder->initialized) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return 0.0f;
    }

    return encoder->last_speed_rps;
}

/**
 * @brief Get cached speed in radians per second
 */
float encoder_get_speed_rad_s(encoder_handle_t handle)
{
    if (handle == NULL) {
        return 0.0f;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (!encoder->initialized) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return 0.0f;
    }

    return encoder->last_speed_rad_s;
}

/**
 * @brief Reset encoder count to zero
 */
esp_err_t encoder_reset_count(encoder_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;

    if (!encoder->initialized) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return ESP_FAIL;
    }

    esp_err_t ret = pcnt_counter_clear(encoder->config.pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT counter: %s", esp_err_to_name(ret));
        return ret;
    }

    encoder->current_count = 0;
    encoder->last_count = 0;

    ESP_LOGD(TAG, "Encoder count reset");

    return ESP_OK;
}

/**
 * @brief Get encoder configuration
 */
esp_err_t encoder_get_config(encoder_handle_t handle, encoder_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    encoder_context_t *encoder = (encoder_context_t *)handle;
    memcpy(config, &encoder->config, sizeof(encoder_config_t));

    return ESP_OK;
}

/**
 * @brief Convert encoder state to JSON object
 */
cJSON* encoder_state_to_json(encoder_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid encoder handle");
        return NULL;
    }

    encoder_state_t state;
    esp_err_t ret = encoder_get_state(handle, &state);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get encoder state for JSON");
        return NULL;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return NULL;
    }

    // Add encoder data to JSON
    cJSON_AddNumberToObject(root, "count", state.count);
    cJSON_AddNumberToObject(root, "delta", state.delta);
    cJSON_AddNumberToObject(root, "speed_rpm", state.speed_rpm);
    cJSON_AddNumberToObject(root, "speed_rps", state.speed_rps);
    cJSON_AddNumberToObject(root, "speed_rad_s", state.speed_rad_s);
    cJSON_AddNumberToObject(root, "timestamp_ms", state.timestamp_ms);

    // Add configuration info
    encoder_config_t config;
    if (encoder_get_config(handle, &config) == ESP_OK) {
        cJSON *config_obj = cJSON_CreateObject();
        if (config_obj != NULL) {
            cJSON_AddNumberToObject(config_obj, "pin_a", config.pin_a);
            cJSON_AddNumberToObject(config_obj, "pin_b", config.pin_b);
            cJSON_AddNumberToObject(config_obj, "pcnt_unit", config.pcnt_unit);
            cJSON_AddNumberToObject(config_obj, "ppr", config.ppr);
            cJSON_AddItemToObject(root, "config", config_obj);
        }
    }

    ESP_LOGD(TAG, "Encoder state converted to JSON");

    return root;
}

/**
 * @brief Get encoder state as JSON string
 */
char* encoder_state_to_json_string(encoder_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid encoder handle");
        return NULL;
    }

    cJSON *json_obj = encoder_state_to_json(handle);
    if (json_obj == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return NULL;
    }

    // Create formatted JSON string
    char *json_string = cJSON_Print(json_obj);
    cJSON_Delete(json_obj);

    if (json_string == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON string");
        return NULL;
    }

    ESP_LOGD(TAG, "JSON string: %s", json_string);

    return json_string;
}

/**
 * @brief Get current sensor reading (uptime)
 */
esp_err_t test_sensor_read(test_sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Test sensor not initialized");
        return ESP_FAIL;
    }

    // Calculate elapsed time since boot
    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;

    // Convert to different units
    data->uptime_ms = elapsed_us / 1000;
    data->uptime_sec = elapsed_us / 1000000;
    data->uptime_min = (elapsed_us / 1000000) / 60;

    ESP_LOGD(TAG, "Uptime: %lu ms, %lu s, %lu min", 
             data->uptime_ms, data->uptime_sec, data->uptime_min);

    return ESP_OK;
}

/**
 * @brief Get uptime in milliseconds
 */
uint32_t test_sensor_get_uptime_ms(void)
{
    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Test sensor not initialized");
        return 0;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;
    return (uint32_t)(elapsed_us / 1000);
}

/**
 * @brief Get uptime in seconds
 */
uint32_t test_sensor_get_uptime_sec(void)
{
    if (boot_time_us == 0) {
        ESP_LOGW(TAG, "Test sensor not initialized");
        return 0;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;
    return (uint32_t)(elapsed_us / 1000000);
}

/**
 * @brief Get uptime as formatted string
 */
const char *test_sensor_get_uptime_str(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 16) {
        return "ERROR";
    }

    if (boot_time_us == 0) {
        snprintf(buffer, buffer_size, "NOT_INITIALIZED");
        return buffer;
    }

    int64_t current_time_us = esp_timer_get_time();
    int64_t elapsed_us = current_time_us - boot_time_us;
    
    uint32_t total_sec = elapsed_us / 1000000;
    uint32_t hours = total_sec / 3600;
    uint32_t minutes = (total_sec % 3600) / 60;
    uint32_t seconds = total_sec % 60;
    uint32_t milliseconds = (elapsed_us % 1000000) / 1000;

    snprintf(buffer, buffer_size, "%luh %lum %lus %lums", 
             hours, minutes, seconds, milliseconds);

    return buffer;
}
