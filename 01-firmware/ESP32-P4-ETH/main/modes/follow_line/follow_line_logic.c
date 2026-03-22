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
};

static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

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

esp_err_t follow_line_logic_update(follow_line_logic_handle_t handle, 
                                   const follow_line_logic_input_t* input, 
                                   follow_line_logic_output_t* out_output,
                                   float dt_s) {
    if (handle == NULL || input == NULL || out_output == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = handle;

    if (!input->line_detected) {
        ctx->integral = 0;
        if (ctx->last_known_position < -0.01f) {
            out_output->left_motor_speed = -input->base_speed;
            out_output->right_motor_speed = input->base_speed;
        } else if (ctx->last_known_position > 0.01f) {
            out_output->left_motor_speed = input->base_speed;
            out_output->right_motor_speed = -input->base_speed;
        } else {
            out_output->left_motor_speed = input->base_speed;
            out_output->right_motor_speed = input->base_speed;
        }
        out_output->p_term = 0;
        out_output->i_term = 0;
        out_output->d_term = 0;
        out_output->raw_steering = 0;
        return ESP_OK;
    }

    ctx->last_known_position = input->line_position;
    float error = input->line_position;
    ctx->integral += error * dt_s;
    float derivative = (error - ctx->previous_error) / dt_s;
    
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

esp_err_t follow_line_logic_set_config(follow_line_logic_handle_t handle, const follow_line_logic_config_t* config) {
    if (handle == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    struct follow_line_logic_context_t* ctx = handle;
    ctx->config = *config;
    ctx->integral = 0; // Reset windup on config change
    return ESP_OK;
}
