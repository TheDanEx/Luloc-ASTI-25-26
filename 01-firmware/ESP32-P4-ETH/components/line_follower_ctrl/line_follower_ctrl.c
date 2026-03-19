#include "line_follower_ctrl.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "LINE_FOLLOWER_CTRL";

// =============================================================================
// Private Helpers
// =============================================================================

/**
 * Clamp a float value between min and max.
 */
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// =============================================================================
// Types and Context
// =============================================================================

struct line_follower_ctrl_context_t {
    line_follower_config_t config;
    
    // PID State
    float integral;
    float previous_error;
    
    // Memory for advanced logic
    float last_known_position; // To remember which side we lost the line from
};

// =============================================================================
// Public API: Lifecycle
// =============================================================================

/**
 * Initialize a new line follower controller.
 * Configures PID gains, speed limits, and camera feed-forward weights.
 */
esp_err_t line_follower_ctrl_create(const line_follower_config_t* config, line_follower_ctrl_handle_t* out_handle) {
    if (config == NULL || out_handle == NULL) return ESP_ERR_INVALID_ARG;

    struct line_follower_ctrl_context_t* ctx = calloc(1, sizeof(struct line_follower_ctrl_context_t));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for line follower controller");
        return ESP_ERR_NO_MEM;
    }

    ctx->config = *config;
    ctx->integral = 0.0f;
    ctx->previous_error = 0.0f;
    ctx->last_known_position = 0.0f;

    *out_handle = ctx;
    ESP_LOGI(TAG, "Line Follower Controller created successfully.");
    return ESP_OK;
}

/**
 * Teardown the controller instance.
 */
esp_err_t line_follower_ctrl_destroy(line_follower_ctrl_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    free(handle);
    return ESP_OK;
}

// =============================================================================
// Public API: Control Implementation
// =============================================================================

/**
 * Main control loop for line following.
 * Logic:
 * 1. Lost-Line recovery: If no sensor detects the line, use last known side to spin-search.
 * 2. PID Steering: Traditional proportional-integral-derivative based on sensor centroid.
 * 3. Camera Feed-Forward: Blends predictive steering from a camera module for look-ahead curves.
 * 4. Differential Drive Output: Converts steering commands to individual wheel target speeds.
 */
esp_err_t line_follower_ctrl_update(line_follower_ctrl_handle_t handle, 
                                     const line_follower_input_t* input, 
                                     line_follower_output_t* out_output,
                                     float delta_time_s) {
    if (handle == NULL || input == NULL || out_output == NULL) return ESP_ERR_INVALID_ARG;
    
    if (delta_time_s <= 0.0f) delta_time_s = 0.01f;

    struct line_follower_ctrl_context_t* ctx = handle;

    // --- 1. Recovery Logic (Lost Line) ---
    if (!input->line_detected) {
        // Line is lost. Decide what to do based on the last known position.
        ctx->integral = 0; // Reset anti-windup when lost

        if (ctx->last_known_position < -0.4f) {
            // Lost line to the extreme left. Spin to find it.
            out_output->left_motor_speed = -ctx->config.base_speed;
            out_output->right_motor_speed = ctx->config.base_speed;
            ESP_LOGD(TAG, "Line lost to Left. Spinning left.");
            return ESP_OK;
        } 
        else if (ctx->last_known_position > 0.4f) {
            // Lost line to the extreme right. Spin right.
            out_output->left_motor_speed = ctx->config.base_speed;
            out_output->right_motor_speed = -ctx->config.base_speed;
            ESP_LOGD(TAG, "Line lost to Right. Spinning right.");
            return ESP_OK;
        } 
        else {
            // Gap/Intersection: Use camera ff or go straight
            // This happens at diamond crossings or track gaps ("Siga recto si puede").
            float feed_forward = input->camera_curvature * ctx->config.camera_weight;
            out_output->left_motor_speed = clamp(ctx->config.base_speed + feed_forward, -ctx->config.max_speed, ctx->config.max_speed);
            out_output->right_motor_speed = clamp(ctx->config.base_speed - feed_forward, -ctx->config.max_speed, ctx->config.max_speed);
            
            ESP_LOGD(TAG, "Line lost in center (intersection). Going straight/camera-guided.");
            return ESP_OK;
        }
    }

    // --- 2. Active Pursuit ---
    ctx->last_known_position = input->line_position;
    float error = input->line_position;
    
    ctx->integral += error * delta_time_s;
    float derivative = (error - ctx->previous_error) / delta_time_s;
    
    // Steering contributions
    float pid_steering = (ctx->config.kp * error) + 
                         (ctx->config.ki * ctx->integral) + 
                         (ctx->config.kd * derivative);
                         
    float feed_forward_steering = input->camera_curvature * ctx->config.camera_weight;
    
    float total_steering = pid_steering + feed_forward_steering;
    
    // --- 3. Mixing & Saturation ---
    float left_speed = ctx->config.base_speed + total_steering;
    float right_speed = ctx->config.base_speed - total_steering;

    out_output->left_motor_speed = clamp(left_speed, -ctx->config.max_speed, ctx->config.max_speed);
    out_output->right_motor_speed = clamp(right_speed, -ctx->config.max_speed, ctx->config.max_speed);

    ctx->previous_error = error;
    return ESP_OK;
}

// =============================================================================
// Public API: Configuration
// =============================================================================

/**
 * Live update of steering PID constants.
 */
esp_err_t line_follower_ctrl_set_pid(line_follower_ctrl_handle_t handle, float kp, float ki, float kd) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    
    struct line_follower_ctrl_context_t* ctx = handle;
    ctx->config.kp = kp;
    ctx->config.ki = ki;
    ctx->config.kd = kd;
    
    ctx->integral = 0;
    return ESP_OK;
}

/**
 * Adjust the influence of the camera-based predictive steering.
 */
esp_err_t line_follower_ctrl_set_camera_weight(line_follower_ctrl_handle_t handle, float weight) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    
    struct line_follower_ctrl_context_t* ctx = handle;
    ctx->config.camera_weight = weight;
    return ESP_OK;
}
