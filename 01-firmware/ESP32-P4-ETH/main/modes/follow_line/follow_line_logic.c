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
    int8_t last_turn_dir; // -1 left, +1 right
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

    if (!input->line_detected) {
        ctx->integral = 0.0f;

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

        // Search behavior: rotate toward the last side where the line was seen.
        // Thresholds adjusted for millimeters (e.g. 10mm array half-width).
        int8_t search_dir = ctx->last_turn_dir;
        if (ctx->last_known_position < -10.0f) {
            search_dir = -1;
        } else if (ctx->last_known_position > 10.0f) {
            search_dir = 1;
        }
        if (search_dir == 0) {
            search_dir = 1;
        }

        float spin_speed = clamp(input->base_speed * 0.7f, 0.08f, ctx->config.max_speed);
        out_output->left_motor_speed = (search_dir < 0) ? -spin_speed : spin_speed;
        out_output->right_motor_speed = -out_output->left_motor_speed;

        out_output->p_term = 0.0f;
        out_output->i_term = 0.0f;
        out_output->d_term = 0.0f;
        out_output->raw_steering = 0.0f;
        return ESP_OK;
    }

    ctx->has_seen_line = true;
    ctx->last_known_position = input->line_position_mm;

    float safe_dt = (dt_s > 0.0001f) ? dt_s : 0.0001f;
    float error = input->line_position_mm;

    ctx->integral += error * safe_dt;
    ctx->integral = clamp(ctx->integral, -1.5f, 1.5f);

    float derivative = (error - ctx->previous_error) / safe_dt;

    // Hysteresis for turn direction memory
    if (error > 2.0f) {
        ctx->last_turn_dir = 1;
    } else if (error < -2.0f) {
        ctx->last_turn_dir = -1;
    }

    float p_term = ctx->config.kp * error;
    float i_term = ctx->config.ki * ctx->integral;
    float d_term = ctx->config.kd * derivative;
    float total_steering = p_term + i_term + d_term;

    out_output->left_motor_speed = clamp(input->base_speed + total_steering, -ctx->config.max_speed, ctx->config.max_speed);
    out_output->right_motor_speed = clamp(input->base_speed - total_steering, -ctx->config.max_speed, ctx->config.max_speed);

    // Diagnostics
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
