#include "follow_line_logic.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const char *TAG = "FOLLOW_LINE_LOGIC";

// =============================================================================
// Private Context
// =============================================================================

struct follow_line_logic_context_t {
    follow_line_logic_config_t config;
    float integral;
    float previous_error;
    float last_known_position;
    bool has_seen_line;
    int8_t last_exit_side; // -1 = line exited left, +1 = line exited right
};

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// =============================================================================
// Public API: Lifecycle
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
// Public API: Execution
// =============================================================================

/**
 * Core update loop. Three operating regions:
 *
 * 1. LINE VISIBLE  → Standard PID on real centroid. Reset integral when centered.
 * 2. LINE LOST     → Virtual sensor injects a large error on the exit side.
 *                     Uses ki_search instead of ki so the integral ramps harder.
 * 3. NEVER SEEN    → Full stop (safety).
 */
esp_err_t follow_line_logic_update(follow_line_logic_handle_t handle,
                                   const follow_line_logic_input_t* input,
                                   follow_line_logic_output_t* out_output,
                                   float dt_s) {
    if (handle == NULL || input == NULL || out_output == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = handle;

    float safe_dt = (dt_s > 0.0001f) ? dt_s : 0.0001f;
    float error = 0.0f;
    float active_ki = ctx->config.ki;
    bool searching = false;

    // -------------------------------------------------------------------------
    // Region 3: Never seen a line — do NOT drive blind
    // -------------------------------------------------------------------------
    if (!input->line_detected && !ctx->has_seen_line) {
        out_output->left_motor_speed  = 0.0f;
        out_output->right_motor_speed = 0.0f;
        out_output->p_term       = 0.0f;
        out_output->i_term       = 0.0f;
        out_output->d_term       = 0.0f;
        out_output->raw_steering = 0.0f;
        out_output->is_searching = false;
        return ESP_OK;
    }

    // -------------------------------------------------------------------------
    // Region 1: Line detected — normal PID
    // -------------------------------------------------------------------------
    if (input->line_detected) {
        ctx->has_seen_line = true;
        ctx->last_known_position = input->line_position_mm;
        error = input->line_position_mm;

        // Track exit side with hysteresis (> 2mm offset to avoid jitter)
        if (error > 2.0f) {
            ctx->last_exit_side = 1;
        } else if (error < -2.0f) {
            ctx->last_exit_side = -1;
        }

        // Reset integral when the robot is centered (within deadband)
        if (fabsf(error) < ctx->config.center_deadband_mm) {
            ctx->integral = 0.0f;
        }

        searching = false;
    }
    // -------------------------------------------------------------------------
    // Region 2: Line lost — inject virtual sensor error
    // -------------------------------------------------------------------------
    else {
        // Determine which side the line exited from
        int8_t exit_side = ctx->last_exit_side;
        if (exit_side == 0) {
            // Fallback: infer from last known position
            exit_side = (ctx->last_known_position >= 0.0f) ? 1 : -1;
        }

        // Virtual sensor: a fictitious reading far beyond the real array edge.
        // The sign matches the exit side so the PID steers TOWARD the line.
        error = exit_side * ctx->config.virtual_sensor_offset_mm;

        // Switch to the search-mode integral gain (typically stronger)
        active_ki = ctx->config.ki_search;
        searching = true;
    }

    // -------------------------------------------------------------------------
    // Unified PID Computation
    // -------------------------------------------------------------------------
    ctx->integral += error * safe_dt;
    ctx->integral = clamp(ctx->integral, -1.5f, 1.5f);

    float derivative = (error - ctx->previous_error) / safe_dt;

    float p_term = ctx->config.kp * error;
    float i_term = active_ki * ctx->integral;
    float d_term = ctx->config.kd * derivative;
    float total_steering = p_term + i_term + d_term;

    out_output->left_motor_speed  = clamp(input->base_speed + total_steering, -ctx->config.max_speed, ctx->config.max_speed);
    out_output->right_motor_speed = clamp(input->base_speed - total_steering, -ctx->config.max_speed, ctx->config.max_speed);

    // Diagnostics
    out_output->p_term       = p_term;
    out_output->i_term       = i_term;
    out_output->d_term       = d_term;
    out_output->raw_steering = total_steering;
    out_output->is_searching = searching;

    ctx->previous_error = error;
    return ESP_OK;
}

// =============================================================================
// Public API: Configuration
// =============================================================================

esp_err_t follow_line_logic_set_config(follow_line_logic_handle_t handle, const follow_line_logic_config_t* config) {
    if (handle == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = handle;
    ctx->config = *config;
    ctx->integral = 0.0f; // Reset windup on config change
    return ESP_OK;
}