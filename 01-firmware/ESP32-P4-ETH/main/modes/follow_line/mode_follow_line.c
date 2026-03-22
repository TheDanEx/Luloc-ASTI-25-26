#include "mode_interface.h"
#include "esp_log.h"
#include "motor.h"
#include "shared_memory.h"
#include "follow_line_logic.h"
#include "mqtt_custom_client.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

#include "telemetry_manager.h"

static const char *TAG = "MODE_FOLLOW_LINE";
static follow_line_logic_handle_t s_logic = NULL;
static telemetry_handle_t s_telemetry = NULL;
static volatile float s_curvature_multiplier = 1.0f; // Default: No change

// Default configuration from Kconfig
static follow_line_logic_config_t s_current_config = {
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .max_speed = 0.0f
};
static float s_base_speed_nominal = 0.0f;
static float s_ff_weight = 0.0f;
static bool s_defaults_loaded = false;

#define CURVATURE_TOPIC "robot/vision/curvature"
#define CONFIG_TOPIC    "robot/config/follow_line"

/**
 * MQTT Callback for real-time config updates (JSON)
 */
static void mqtt_config_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (data == NULL || data_len <= 0) return;
    
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) return;

    cJSON *kp = cJSON_GetObjectItem(root, "kp");
    cJSON *ki = cJSON_GetObjectItem(root, "ki");
    cJSON *kd = cJSON_GetObjectItem(root, "kd");
    cJSON *max = cJSON_GetObjectItem(root, "max_speed");
    cJSON *ffw = cJSON_GetObjectItem(root, "ff_weight");

    if (kp) s_current_config.kp = kp->valuedouble;
    if (ki) s_current_config.ki = ki->valuedouble;
    if (kd) s_current_config.kd = kd->valuedouble;
    if (max) s_current_config.max_speed = max->valuedouble;
    if (ffw) s_ff_weight = ffw->valuedouble;

    if (s_logic) {
        follow_line_logic_set_config(s_logic, &s_current_config);
        ESP_LOGI(TAG, "Dynamic Config Updated: P=%.2f I=%.2f D=%.2f Max=%.2f FFw=%.2f", 
                 s_current_config.kp, s_current_config.ki, s_current_config.kd, 
                 s_current_config.max_speed, s_ff_weight);
    }

    cJSON_Delete(root);
}

/**
 * MQTT Callback for curvature updates
 */
static void mqtt_curvature_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (data == NULL || data_len <= 0) return;
    
    char payload[32] = {0};
    int copy_len = (data_len < 31) ? data_len : 31;
    memcpy(payload, data, copy_len);

    char *endptr = NULL;
    float val = strtof(payload, &endptr);
    if (endptr != payload) {
        // We assume the RPi sends a multiplier (e.g. 0.8 to slow down)
        // or a curvature where we calculate the multiplier.
        // For now, we take it as a direct speed multiplier.
        s_curvature_multiplier = val;
    }
}

static void enter(void) {
    ESP_LOGI(TAG, "Entering FOLLOW_LINE mode");
    
    // 0. Load defaults from Kconfig only once. Live MQTT config can override later.
    if (!s_defaults_loaded) {
        s_current_config.kp = atof(CONFIG_FOLLOW_LINE_KP);
        s_current_config.ki = atof(CONFIG_FOLLOW_LINE_KI);
        s_current_config.kd = atof(CONFIG_FOLLOW_LINE_KD);
        s_current_config.max_speed = atof(CONFIG_FOLLOW_LINE_MAX_SPEED);
        s_base_speed_nominal = atof(CONFIG_FOLLOW_LINE_BASE_SPEED);
        s_ff_weight = atof(CONFIG_FOLLOW_LINE_FF_WEIGHT);
        s_defaults_loaded = true;

        ESP_LOGI(TAG, "Loaded default follow_line config: P=%.2f I=%.2f D=%.2f Base=%.2f Max=%.2f FFw=%.2f",
                 s_current_config.kp, s_current_config.ki, s_current_config.kd,
                 s_base_speed_nominal, s_current_config.max_speed, s_ff_weight);
    }

    // 1. Initialize logic with static or last known config
    follow_line_logic_create(&s_current_config, &s_logic);

    // 2. Register MQTT callbacks
    mqtt_custom_client_register_topic_callback(CURVATURE_TOPIC, mqtt_curvature_callback);
    mqtt_custom_client_register_topic_callback(CONFIG_TOPIC,    mqtt_config_callback);
    
    // 3. Initialize Telemetry
    if (s_telemetry == NULL) {
        s_telemetry = telemetry_create("robot/telemetry/follow_line", "line_follower", CONFIG_TELEMETRY_INTERVAL_FOLLOW_LINE_MS);
    }

    if (mqtt_custom_client_is_connected()) {
        mqtt_custom_client_subscribe(CURVATURE_TOPIC, 0);
        mqtt_custom_client_subscribe(CONFIG_TOPIC, 0);
    }
}

static void execute(motor_driver_mcpwm_t* motors, 
                    motor_velocity_ctrl_handle_t ctrl_left, 
                    motor_velocity_ctrl_handle_t ctrl_right, 
                    float dt_s) 
{
    if (s_logic == NULL) return;

    shared_memory_t* shm = shared_memory_get();
    
    // 1. Read Inputs
    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    float line_pos = shm->sensors.line_position;
    bool detected = shm->sensors.line_detected;
    float bat_mv = shm->sensors.battery_voltage;
    float cur_l = shm->sensors.motor_speed_left;
    float cur_r = shm->sensors.motor_speed_right;
    xSemaphoreGive(shm->mutex);

    if (bat_mv < 5000) bat_mv = 16800;

    // 2. Adjust base speed (RPi curvature multiplier blended with weight)
    float effective_multiplier = 1.0f + (s_curvature_multiplier - 1.0f) * s_ff_weight;
    float dynamic_base_speed = s_base_speed_nominal * effective_multiplier;

    follow_line_logic_input_t input = {
        .line_position = line_pos,
        .line_detected = detected,
        .base_speed = dynamic_base_speed
    };

    // 3. Compute Strategy
    follow_line_logic_output_t output;
    follow_line_logic_update(s_logic, &input, &output, dt_s);

    // 4. Drive Motors via Velocity Controller
    motor_velocity_input_t motor_l = { .target_speed = output.left_motor_speed, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t motor_r = { .target_speed = output.right_motor_speed, .current_speed = cur_r, .battery_mv = bat_mv };

    float pwm_l, pwm_r;
    motor_velocity_ctrl_update(ctrl_left,  &motor_l, dt_s, &pwm_l, NULL);
    motor_velocity_ctrl_update(ctrl_right, &motor_r, dt_s, &pwm_r, NULL);

    motor_mcpwm_set(motors, (int16_t)(pwm_l * 10.0f), (int16_t)(pwm_r * 10.0f));

    // 5. Telemetry
    if (s_telemetry) {
        telemetry_add_float(s_telemetry, "line_pos",      line_pos);
        telemetry_add_bool(s_telemetry,  "line_detected", detected);
        telemetry_add_float(s_telemetry, "base_speed",    dynamic_base_speed);
        telemetry_add_float(s_telemetry, "target_l",      output.left_motor_speed);
        telemetry_add_float(s_telemetry, "target_r",      output.right_motor_speed);
        telemetry_add_float(s_telemetry, "actual_l",      cur_l);
        telemetry_add_float(s_telemetry, "actual_r",      cur_r);
        telemetry_add_float(s_telemetry, "p_term",        output.p_term);
        telemetry_add_float(s_telemetry, "i_term",        output.i_term);
        telemetry_add_float(s_telemetry, "d_term",        output.d_term);
        telemetry_add_float(s_telemetry, "steering",      output.raw_steering);

        // Explicit per-motor PID effects (as requested)
        telemetry_add_float(s_telemetry, "p_eff_l",      output.p_term);
        telemetry_add_float(s_telemetry, "p_eff_r",     -output.p_term);
        telemetry_add_float(s_telemetry, "i_eff_l",      output.i_term);
        telemetry_add_float(s_telemetry, "i_eff_r",     -output.i_term);
        telemetry_add_float(s_telemetry, "d_eff_l",      output.d_term);
        telemetry_add_float(s_telemetry, "d_eff_r",     -output.d_term);

        telemetry_commit_point(s_telemetry);
    }
}

static void exit_mode(motor_driver_mcpwm_t* motors) {
    ESP_LOGI(TAG, "Exiting FOLLOW_LINE mode");
    if (s_logic) {
        follow_line_logic_destroy(s_logic);
        s_logic = NULL;
    }
    if (s_telemetry) {
        telemetry_destroy(s_telemetry);
        s_telemetry = NULL;
    }
    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_follow_line = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};
