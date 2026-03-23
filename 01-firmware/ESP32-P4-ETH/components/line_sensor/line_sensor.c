/*
 * Line Sensor Array Component Implementation
 * SPDX-License-Identifier: MIT
 */

#include "line_sensor.h"
#include <malloc.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Define the private context struct here (Opaque Pointer pattern)
struct line_sensor_context {
    line_sensor_config_t config;
    adc_oneshot_unit_handle_t adc_handle;
    
    uint16_t *internal_raw_buffer;
    float *internal_norm_buffer;
    bool *internal_digital_buffer;

    adc_channel_t *adc_channels_copy;
    float *sensor_positions_copy;
    
    uint16_t *calib_min;
    uint16_t *calib_max;
    
    volatile bool is_calibrating;
    volatile bool is_fully_calibrated;
    float current_centroid;
    
    TaskHandle_t calib_task_handle;
    SemaphoreHandle_t mutex;
    
    bool is_initialized;
};

static const char *TAG = "line_sensor";

// =============================================================================
// INTERNAL HELPERS
// =============================================================================

/**
 * Read a single ADC channel multiple times and return the arithmetic mean.
 * This filters high-frequency electrical noise from the IR sensors.
 */
static esp_err_t read_sensor_averaged(struct line_sensor_context *ctx, int sensor_idx, uint16_t *out_val)
{
    uint32_t accumulator = 0;
    int raw_val = 0;
    
    for (int i = 0; i < ctx->config.oversample_count; i++) {
        esp_err_t err = adc_oneshot_read(ctx->adc_handle, ctx->config.adc_channels[sensor_idx], &raw_val);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed (unit=%d idx=%d ch=%d err=0x%x)",
                     (int)ctx->config.adc_unit,
                     sensor_idx,
                     (int)ctx->config.adc_channels[sensor_idx],
                     (unsigned int)err);
            return err;
        }
        accumulator += raw_val;
    }
    
    *out_val = (uint16_t)(accumulator / ctx->config.oversample_count);
    return ESP_OK;
}

/**
 * Background calibration task (Core 1).
 * Continuously samples all sensors to find the dynamic range (Min/Max).
 * The user should move the robot laterally over the line during this phase.
 */
static void calibration_task(void *arg)
{
    struct line_sensor_context *ctx = (struct line_sensor_context *)arg;
    
    ESP_LOGI(TAG, "Calibration task started. Move the sensor laterally over the line...");
    
    while(ctx->is_calibrating) {
        xSemaphoreTake(ctx->mutex, portMAX_DELAY);
        
        bool all_calibrated_so_far = true;
        
        for (int i = 0; i < ctx->config.num_sensors; i++) {
            uint16_t val = 0;
            if (read_sensor_averaged(ctx, i, &val) == ESP_OK) {
                // Adjust dynamic bounds
                if (val < ctx->calib_min[i]) ctx->calib_min[i] = val;
                if (val > ctx->calib_max[i]) ctx->calib_max[i] = val;
                
                // Sensor is calibrated if it has seen enough contrast (based on threshold)
                if ((ctx->calib_max[i] - ctx->calib_min[i]) < ctx->config.calibration_threshold) {
                    all_calibrated_so_far = false;
                }
            }
        }
        
        // Final flag flips only when EVERY sensor is ready
        ctx->is_fully_calibrated = all_calibrated_so_far;
        
        xSemaphoreGive(ctx->mutex);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    ESP_LOGI(TAG, "Calibration task stopped.");
    ctx->calib_task_handle = NULL;
    vTaskDelete(NULL);
}

// =============================================================================
// PUBLIC API: LIFECYCLE
// =============================================================================

/**
 * Create a new line sensor instance from ADC configuration.
 * Allocates internal buffers and initializes the oneshot ADC unit.
 */
line_sensor_handle_t line_sensor_init(const line_sensor_config_t *config)
{
    if (config == NULL || config->num_sensors <= 0 || config->adc_channels == NULL) {
        ESP_LOGE(TAG, "Invalid configuration parameters");
        return NULL;
    }

    struct line_sensor_context *ctx = (struct line_sensor_context *)calloc(1, sizeof(struct line_sensor_context));
    if (ctx == NULL) return NULL;

    memcpy(&ctx->config, config, sizeof(line_sensor_config_t));

    // Deep-copy channel/position arrays to avoid dangling pointers.
    ctx->adc_channels_copy = (adc_channel_t *)calloc(config->num_sensors, sizeof(adc_channel_t));
    if (ctx->adc_channels_copy == NULL) {
        free(ctx);
        return NULL;
    }
    memcpy(ctx->adc_channels_copy, config->adc_channels, sizeof(adc_channel_t) * config->num_sensors);
    ctx->config.adc_channels = ctx->adc_channels_copy;

    if (config->sensor_positions_m != NULL) {
        ctx->sensor_positions_copy = (float *)calloc(config->num_sensors, sizeof(float));
        if (ctx->sensor_positions_copy == NULL) {
            free(ctx->adc_channels_copy);
            free(ctx);
            return NULL;
        }
        memcpy(ctx->sensor_positions_copy, config->sensor_positions_m, sizeof(float) * config->num_sensors);
        ctx->config.sensor_positions_m = ctx->sensor_positions_copy;
    } else {
        ctx->sensor_positions_copy = NULL;
        ctx->config.sensor_positions_m = NULL;
    }
    
    // Apply defaults from Kconfig if applicable
    if (ctx->config.oversample_count == 0) {
        #ifdef CONFIG_LINE_SENSOR_OVERSAMPLING
        ctx->config.oversample_count = CONFIG_LINE_SENSOR_OVERSAMPLING;
        #else
        ctx->config.oversample_count = 16;
        #endif
    }
    if (ctx->config.calibration_threshold == 0) {
        #ifdef CONFIG_LINE_SENSOR_CALIB_THRESHOLD
        ctx->config.calibration_threshold = CONFIG_LINE_SENSOR_CALIB_THRESHOLD;
        #else
        ctx->config.calibration_threshold = 1500;
        #endif
    }
    if (ctx->config.detection_threshold <= 0.001f) {
        #ifdef CONFIG_LINE_SENSOR_DETECTION_THRESHOLD
        ctx->config.detection_threshold = (float)CONFIG_LINE_SENSOR_DETECTION_THRESHOLD / 100.0f;
        #else
        ctx->config.detection_threshold = 0.5f;
        #endif
    }

    // Defensive buffer allocation
    ctx->internal_raw_buffer = (uint16_t *)calloc(config->num_sensors, sizeof(uint16_t));
    ctx->internal_norm_buffer = (float *)calloc(config->num_sensors, sizeof(float));
    ctx->internal_digital_buffer = (bool *)calloc(config->num_sensors, sizeof(bool));
    ctx->calib_min = (uint16_t *)calloc(config->num_sensors, sizeof(uint16_t));
    ctx->calib_max = (uint16_t *)calloc(config->num_sensors, sizeof(uint16_t));
    
    if (!ctx->internal_raw_buffer || !ctx->internal_norm_buffer || !ctx->internal_digital_buffer || !ctx->calib_min || !ctx->calib_max || !ctx->adc_channels_copy) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor buffers");
        line_sensor_deinit((line_sensor_handle_t)ctx);
        return NULL;
    }

    // Preparation for dynamic range search
    for (int i = 0; i < config->num_sensors; i++) {
        ctx->calib_min[i] = 4095;
        ctx->calib_max[i] = 0;
    }

    ctx->mutex = xSemaphoreCreateMutex();
    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = config->adc_unit,
    };
    if (adc_oneshot_new_unit(&init_config1, &ctx->adc_handle) != ESP_OK) {
         ESP_LOGE(TAG, "Failed to initialize ADC unit");
         line_sensor_deinit((line_sensor_handle_t)ctx);
         return NULL;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < config->num_sensors; i++) {
        esp_err_t cfg_err = adc_oneshot_config_channel(ctx->adc_handle, config->adc_channels[i], &channel_config);
        if (cfg_err != ESP_OK) {
             ESP_LOGE(TAG, "Failed to config ADC channel for sensor %d (unit=%d ch=%d err=0x%x)",
                      i, (int)ctx->config.adc_unit, (int)config->adc_channels[i], (unsigned int)cfg_err);
             line_sensor_deinit((line_sensor_handle_t)ctx);
             return NULL;
        }
    }

    ESP_LOGI(TAG, "Line Sensor Array initialized [%d sensors | unit=%d | Threshold: %d]", 
             config->num_sensors, (int)ctx->config.adc_unit, (int)ctx->config.calibration_threshold);

    ctx->is_initialized = true;
    return (line_sensor_handle_t)ctx;
}

/**
 * Teardown resources and free buffers.
 * Stops any active calibration task before cleanup.
 */
esp_err_t line_sensor_deinit(line_sensor_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;
    
    line_sensor_calibration_stop(handle);

    if (ctx->adc_handle) {
        adc_oneshot_del_unit(ctx->adc_handle);
    }

    if (ctx->mutex) vSemaphoreDelete(ctx->mutex);
    if (ctx->internal_raw_buffer) free(ctx->internal_raw_buffer);
    if (ctx->internal_norm_buffer) free(ctx->internal_norm_buffer);
    if (ctx->internal_digital_buffer) free(ctx->internal_digital_buffer);
    if (ctx->calib_min) free(ctx->calib_min);
    if (ctx->calib_max) free(ctx->calib_max);
    if (ctx->adc_channels_copy) free(ctx->adc_channels_copy);
    if (ctx->sensor_positions_copy) free(ctx->sensor_positions_copy);
    
    free(ctx);
    ESP_LOGI(TAG, "Line Sensor Array de-initialized");
    return ESP_OK;
}

// =============================================================================
// PUBLIC API: CALIBRATION
// =============================================================================

/**
 * Launch the asynchronous calibration task.
 * Note: Blocking mutex while resetting bounds to ensure thread-safety.
 */
esp_err_t line_sensor_calibration_start(line_sensor_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;

    if (ctx->is_calibrating) return ESP_OK;

    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    ctx->is_calibrating = true;
    ctx->is_fully_calibrated = false;
    
    for (int i = 0; i < ctx->config.num_sensors; i++) {
        ctx->calib_min[i] = 4095;
        ctx->calib_max[i] = 0;
    }
    xSemaphoreGive(ctx->mutex);

    BaseType_t res = xTaskCreatePinnedToCore(
        calibration_task, "calib_task", 3072, ctx, 5, &ctx->calib_task_handle, 1
    );

    if (res != pdPASS) {
        ctx->is_calibrating = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * Stop the calibration task.
 * Sets a flag for the task to exit gracefully.
 */
esp_err_t line_sensor_calibration_stop(line_sensor_handle_t handle)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;

    ctx->is_calibrating = false; 
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

/**
 * Check if all sensors have seen enough contrast to be considered calibrated.
 */
bool line_sensor_is_calibrated(line_sensor_handle_t handle)
{
    if (handle == NULL) return false;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;
    
    return ctx->is_fully_calibrated;
}

// =============================================================================
// PUBLIC API: DATA ACQUISITION
// =============================================================================

/**
 * Read raw ADC counts for all sensors.
 */
esp_err_t line_sensor_read_raw(line_sensor_handle_t handle, uint16_t *out_raw)
{
    if (handle == NULL || out_raw == NULL) return ESP_ERR_INVALID_ARG;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;

    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    for (int i = 0; i < ctx->config.num_sensors; i++) {
        read_sensor_averaged(ctx, i, &out_raw[i]);
    }
    xSemaphoreGive(ctx->mutex);
    return ESP_OK;
}

/**
 * Read normalized reflection values (0.0 to 1.0).
 * Maps raw counts to the dynamic range discovered during calibration.
 */
esp_err_t line_sensor_read_normalized(line_sensor_handle_t handle, float *out_normalized)
{
    if (handle == NULL || out_normalized == NULL) return ESP_ERR_INVALID_ARG;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;

    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    for (int i = 0; i < ctx->config.num_sensors; i++) {
        uint16_t raw_val = 0;
        read_sensor_averaged(ctx, i, &raw_val);
        ctx->internal_raw_buffer[i] = raw_val;
        
        float range = (float)(ctx->calib_max[i] - ctx->calib_min[i]);
        if (range <= 0.0f) range = 1.0f; // Prevent division by zero if uncalibrated
        
        float norm = (float)(raw_val - ctx->calib_min[i]) / range;
        
        // Clamp to 0.0 .. 1.0 (Just in case reading drops below min or above max)
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        
        out_normalized[i] = norm;
    }
    xSemaphoreGive(ctx->mutex);
    
    return ESP_OK;
}

/**
 * Unified read function.
 * Calculates the line centroid using a weighted average of sensor positions.
 * Also performs digital thresholding for state machine integration.
 */
esp_err_t line_sensor_read(line_sensor_handle_t handle, line_sensor_data_t *out_data)
{
    if (handle == NULL || out_data == NULL) return ESP_ERR_INVALID_ARG;
    struct line_sensor_context *ctx = (struct line_sensor_context *)handle;

    // 1. Fills internal_raw_buffer and internal_norm_buffer at the same time
    esp_err_t err = line_sensor_read_normalized(handle, ctx->internal_norm_buffer);
    if (err != ESP_OK) return err;

    // 2. Calculate Centroid (Weighted Average of physical position)
    float sum_weights = 0.0f;
    float sum_positions = 0.0f;
    bool any_detected = false;

    // We assume ctx->config.sensor_positions_m array exists and matches num_sensors
    for (int i = 0; i < ctx->config.num_sensors; i++) {
        float reflectance = ctx->internal_norm_buffer[i];

        #if CONFIG_LINE_SENSOR_LINE_IS_DARK
        float line_signal = 1.0f - reflectance;
        #else
        float line_signal = reflectance;
        #endif

        // Digital conversion (Boolean Array for state machine logic)
        bool is_seeing_line = (line_signal > ctx->config.detection_threshold);
        ctx->internal_digital_buffer[i] = is_seeing_line;

        if (is_seeing_line) {
            any_detected = true;
        }

        // Physical Interpolation (Noise Gate: ignores weak line response)
        if (line_signal > 0.05f) {
            sum_weights += line_signal;

            // If physical distances were provided in config, use them. Otherwise default to flat indices.
            float pos_m = (ctx->config.sensor_positions_m) ? ctx->config.sensor_positions_m[i] : (float)i;
            sum_positions += (line_signal * pos_m);
        }
    }

    if (sum_weights > 0.0f) {
        ctx->current_centroid = sum_positions / sum_weights;
    } else {
        // Line lost, hold last extreme direction
        // For physical metrics, this relies on knowing the extremes.
        // We just hold the last known float value.
    }

    out_data->line_position_mm = ctx->current_centroid;
    out_data->line_detected = any_detected;
    out_data->raw_values = ctx->internal_raw_buffer;
    out_data->normalized_values = ctx->internal_norm_buffer;
    out_data->digital_states = ctx->internal_digital_buffer;

    return ESP_OK;
}
