#include "follow_line_logic.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const char *TAG = "FOLLOW_LINE_LOGIC";

struct follow_line_logic_context_t {
    follow_line_logic_config_t config;
    float integral;
    float last_known_position;
    bool has_seen_line;
    bool was_lost;               // flag: we just lost the line and are searching
    bool was_line_detected;      // prev-frame detection for edge detection
    float heading_rad;           // estimated heading from encoder differential
    float prev_speed_l;
    float prev_speed_r;
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

    // --- Edge detection: line loss / re-acquisition ---
    if (input->line_detected && !ctx->was_line_detected) {
        // Re-acquired after loss: drain accumulated search error
        if (ctx->was_lost) {
            ctx->integral *= 0.2f;
            ctx->heading_rad *= 0.3f;
            ctx->was_lost = false;
            ESP_LOGD(TAG, "Line re-acquired, integral=%f heading=%f", ctx->integral, ctx->heading_rad);
        }
    }
    if (!input->line_detected && ctx->was_line_detected && ctx->has_seen_line) {
        ctx->was_lost = true;
    }
    ctx->was_line_detected = input->line_detected;

    if (input->line_detected) {
        ctx->has_seen_line = true;
        error = input->line_position_m;
        ctx->last_known_position = error;
    } else {
        if (!ctx->has_seen_line) {
            out_output->left_motor_speed = 0.0f;
            out_output->right_motor_speed = 0.0f;
            out_output->p_term = 0.0f;
            out_output->i_term = 0.0f;
            out_output->d_term = 0.0f;
            out_output->ff_term = 0.0f;
            out_output->raw_steering = 0.0f;
            out_output->heading_rad = 0.0f;
            return ESP_OK;
        }
        // Phantom sensor: extrapolate error beyond physical array bounds
        float lost_offset = ctx->config.lost_line_offset_m > 0.001f ? ctx->config.lost_line_offset_m : 0.036f;
        if (ctx->last_known_position < 0.0f) {
            error = -lost_offset;
        } else {
            error =  lost_offset;
        }
    }

    // --- Heading estimation from encoder differential ---
    // ω = (vR - vL) / wheelbase  (positive ω = CCW = turn left)
    float wb = ctx->config.wheelbase_m > 0.01f ? ctx->config.wheelbase_m : 0.17f;
    float avg_speed_l = (input->speed_l + ctx->prev_speed_l) * 0.5f;
    float avg_speed_r = (input->speed_r + ctx->prev_speed_r) * 0.5f;
    float omega = (avg_speed_r - avg_speed_l) / wb;
    ctx->heading_rad += omega * safe_dt;

    // Gentle heading decay when centered and not turning (prevents drift)
    if (fabsf(error) < 0.003f && fabsf(omega) < 0.05f) {
        ctx->heading_rad *= 0.95f;
    }

    ctx->prev_speed_l = input->speed_l;
    ctx->prev_speed_r = input->speed_r;

    // Actual forward speed
    float v_actual = (avg_speed_l + avg_speed_r) * 0.5f;
    float v_nom  = ctx->config.nominal_speed > 0.1f ? ctx->config.nominal_speed : 0.6f;

    // --- PID with speed-invariant gains ---
    //
    // Model:  de/dt = v·θ + L·ω  (L = sensor forward offset)
    //
    // P: constant (acts on position error)
    // I: accumulates e·v·dt (spatial, not temporal)
    // D: acts on heading θ (naturally speed-dependent)
    // FF: v·θ (heading effect × forward speed)

    float p_term = ctx->config.kp * error;

    // Spatial integral: ∫e·v·dt instead of ∫e·dt
    float v_factor = (fabsf(v_actual) > 0.05f) ? (v_actual / v_nom) : 0.05f / v_nom;
    ctx->integral += error * safe_dt * v_factor;
    float i_term = ctx->config.ki * ctx->integral;
    i_term = clamp(i_term, -ctx->config.max_speed, ctx->config.max_speed);

    float d_term = ctx->config.kd * ctx->heading_rad;

    float ff = ctx->config.kff * v_actual * ctx->heading_rad;

    float total_steering = p_term + i_term + d_term + ff;

    out_output->left_motor_speed = clamp(input->base_speed + total_steering, -ctx->config.max_speed, ctx->config.max_speed);
    out_output->right_motor_speed = clamp(input->base_speed - total_steering, -ctx->config.max_speed, ctx->config.max_speed);

    // Diagnostics
    out_output->p_term = p_term;
    out_output->i_term = i_term;
    out_output->d_term = d_term;
    out_output->ff_term = ff;
    out_output->raw_steering = total_steering;
    out_output->heading_rad = ctx->heading_rad;

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