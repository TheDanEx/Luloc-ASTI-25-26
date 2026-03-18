#include "pid_siguelineas.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "pid_linea";

typedef struct {
    pid_siguelineas_config_t cfg;
    bool initialized;
    float integral;
    float prev_error;
    bool prev_valid;
} pid_siguelineas_state_t;

static pid_siguelineas_state_t s_pid = {0};
static const float s_sensor_weights[PID_SIGUELINEAS_NUM_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return (int16_t)v;
}

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static bool read_line_error(float *line_error)
{
    int active_count = 0;
    float weighted_sum = 0.0f;

    for (int i = 0; i < PID_SIGUELINEAS_NUM_SENSORS; i++) {
        gpio_num_t pin = s_pid.cfg.line_sensor_pins[i];
        if (pin == GPIO_NUM_NC) {
            continue;
        }

        int level = gpio_get_level(pin);
        bool active = s_pid.cfg.sensor_active_low ? (level == 0) : (level != 0);
        if (active) {
            weighted_sum += s_sensor_weights[i];
            active_count++;
        }
    }

    if (active_count == 0) {
        return false;
    }

    float error = weighted_sum / (float)active_count;
    *line_error = clamp_f(error / 2.0f, -1.0f, 1.0f);
    return true;
}

esp_err_t pid_siguelineas_init(const pid_siguelineas_config_t *cfg)
{
    if (cfg == NULL || cfg->dt_s <= 0.0f || cfg->max_motor_cmd <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_pid, 0, sizeof(s_pid));
    s_pid.cfg = *cfg;
    s_pid.initialized = true;

    for (int i = 0; i < PID_SIGUELINEAS_NUM_SENSORS; i++) {
        gpio_num_t pin = s_pid.cfg.line_sensor_pins[i];
        if (pin == GPIO_NUM_NC) {
            continue;
        }

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config failed for line sensor pin");
    }

    ESP_LOGI(TAG, "PID siguelineas init: Kp=%.3f Ki=%.3f Kd=%.3f Kff=%.3f base=%d",
             cfg->kp, cfg->ki, cfg->kd, cfg->kff, cfg->base_speed_cmd);
    return ESP_OK;
}

void pid_siguelineas_reset(void)
{
    s_pid.integral = 0.0f;
    s_pid.prev_error = 0.0f;
    s_pid.prev_valid = false;
}

esp_err_t pid_siguelineas_step(float curvature_ff, int16_t *left_cmd, int16_t *right_cmd, pid_siguelineas_debug_t *dbg)
{
    if (!s_pid.initialized || left_cmd == NULL || right_cmd == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    float line_error = 0.0f;
    bool line_valid = read_line_error(&line_error);
    if (!line_valid && s_pid.prev_valid) {
        line_error = s_pid.prev_error;
    }

    float derivative = 0.0f;
    if (line_valid && s_pid.prev_valid) {
        derivative = (line_error - s_pid.prev_error) / s_pid.cfg.dt_s;
    }

    if (line_valid) {
        s_pid.integral += line_error * s_pid.cfg.dt_s;
        s_pid.integral = clamp_f(s_pid.integral, -s_pid.cfg.integral_limit, s_pid.cfg.integral_limit);
        s_pid.prev_error = line_error;
        s_pid.prev_valid = true;
    }

    float pid_u = (s_pid.cfg.kp * line_error) +
                  (s_pid.cfg.ki * s_pid.integral) +
                  (s_pid.cfg.kd * derivative);

    float ff_u = s_pid.cfg.kff * curvature_ff;
    float steer = pid_u + ff_u;
    if (s_pid.cfg.invert_steering) {
        steer = -steer;
    }

    int32_t left = (int32_t)lroundf((float)s_pid.cfg.base_speed_cmd + steer);
    int32_t right = (int32_t)lroundf((float)s_pid.cfg.base_speed_cmd - steer);

    *left_cmd = clamp_i16(left, -s_pid.cfg.max_motor_cmd, s_pid.cfg.max_motor_cmd);
    *right_cmd = clamp_i16(right, -s_pid.cfg.max_motor_cmd, s_pid.cfg.max_motor_cmd);

    if (dbg != NULL) {
        dbg->line_detected = line_valid;
        dbg->line_error = line_error;
        dbg->curvature_ff = curvature_ff;
        dbg->control_u = steer;
        dbg->left_cmd = *left_cmd;
        dbg->right_cmd = *right_cmd;
    }

    return ESP_OK;
}
