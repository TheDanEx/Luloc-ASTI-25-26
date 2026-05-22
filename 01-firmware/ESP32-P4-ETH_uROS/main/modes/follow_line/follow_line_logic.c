#include "follow_line_logic.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "FOLLOW_LINE_LOGIC";

struct follow_line_logic_context_t {
    follow_line_logic_config_t config;
    float integral;
    float previous_error;
    float last_known_position;
    bool has_seen_line;
};

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// =============================================================================
// PUBLIC API: LIFECYCLE
// =============================================================================

esp_err_t follow_line_logic_create(const follow_line_logic_config_t* config, follow_line_logic_handle_t* out_handle) {
    if (config == NULL || out_handle == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = calloc(1, sizeof(struct follow_line_logic_context_t));
    if (ctx == NULL) return ESP_ERR_NO_MEM;
    ctx->config = *config;
    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t follow_line_logic_destroy(follow_line_logic_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    free(handle);
    return ESP_OK;
}

// =============================================================================
// PUBLIC API: EXECUTION
// =============================================================================

esp_err_t follow_line_logic_update(follow_line_logic_handle_t handle,
                                   const follow_line_logic_input_t* input,
                                   follow_line_logic_output_t* out_output,
                                   float dt_s) {
    if (handle == NULL || input == NULL || out_output == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = handle;

    float safe_dt = (dt_s > 0.0001f) ? dt_s : 0.0001f;
    float error = 0.0f;

    if (input->line_detected) {
        ctx->has_seen_line = true;
        error = input->line_position_m;
        ctx->last_known_position = error;
    } else {
        // Safety: before first valid detection, do not drive blind.
        if (!ctx->has_seen_line) {
            out_output->left_motor_speed = 0.0f;
            out_output->right_motor_speed = 0.0f;
            out_output->p_term = 0.0f;
            out_output->i_term = 0.0f;
            out_output->d_term = 0.0f;
            out_output->raw_steering = 0.0f;
            return ESP_OK;
        }

        // Phantom Sensor: extrapola el error cuando se pierde la línea
        if (ctx->last_known_position < 0.0f) {
            error = -0.036f;
        } else {
            error = 0.036f;
        }
    }

    ctx->integral += error * safe_dt;

    float derivative = (error - ctx->previous_error) / safe_dt;

    float p_term = ctx->config.kp * error;
    float i_term = ctx->config.ki * ctx->integral;
    float d_term = ctx->config.kd * derivative;

    float speed_scale = input->base_speed / (ctx->config.nominal_speed > 0.1f ? ctx->config.nominal_speed : 0.6f);
    if (speed_scale < 0.0f) speed_scale = 0.0f;

    float total_steering = (p_term + i_term + d_term) * speed_scale;

    out_output->left_motor_speed = clamp(input->base_speed + total_steering, -ctx->config.max_speed, ctx->config.max_speed);
    out_output->right_motor_speed = clamp(input->base_speed - total_steering, -ctx->config.max_speed, ctx->config.max_speed);

    // Diagnostics (raw terms, before speed scaling — scaled steering is raw_steering)
    out_output->p_term = p_term;
    out_output->i_term = i_term;
    out_output->d_term = d_term;
    out_output->raw_steering = total_steering;

    ctx->previous_error = error;
    return ESP_OK;
}

// =============================================================================
// PUBLIC API: CONFIGURATION
// =============================================================================

esp_err_t follow_line_logic_set_config(follow_line_logic_handle_t handle, const follow_line_logic_config_t* config) {
    if (handle == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = handle;
    ctx->config = *config;
    ctx->integral = 0.0f; // Reset windup on config change
    return ESP_OK;
}