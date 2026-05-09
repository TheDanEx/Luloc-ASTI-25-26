#include "mode_interface.h"
#include "esp_log.h"
#include "shared_memory.h"
#include "telemetry_manager.h"
#include "state_machine.h"
#include "mqtt_custom_client.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODE_CAL";
static telemetry_handle_t cal_telemetry = NULL;

#define CALIBRATION_CONFIG_TOPIC "robot/config/calibration"

typedef struct {
    float speed1;
    float speed2;
    uint32_t interval_ms;
    bool manual_mode;
    float manual_speed;
} sweep_config_t;

static sweep_config_t s_sweep_config = { 0 };

static void mqtt_cal_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (data == NULL || data_len <= 0) return;
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) return;

    cJSON *s1 = cJSON_GetObjectItem(root, "speed1");
    cJSON *s2 = cJSON_GetObjectItem(root, "speed2");
    cJSON *tm = cJSON_GetObjectItem(root, "interval_ms");
    cJSON *man = cJSON_GetObjectItem(root, "manual_mode");
    cJSON *mspd = cJSON_GetObjectItem(root, "manual_speed");

    if (s1) s_sweep_config.speed1 = s1->valuedouble;
    if (s2) s_sweep_config.speed2 = s2->valuedouble;
    if (tm) s_sweep_config.interval_ms = tm->valueint;
    if (man) s_sweep_config.manual_mode = cJSON_IsTrue(man);
    if (mspd) s_sweep_config.manual_speed = mspd->valuedouble;

    ESP_LOGI(TAG, "Sweep Config: Mode=%s S1=%.2f S2=%.2f ManSpd=%.2f", 
             s_sweep_config.manual_mode ? "MANUAL" : "AUTO",
             s_sweep_config.speed1, s_sweep_config.speed2, s_sweep_config.manual_speed);

    cJSON_Delete(root);
}

static float get_sweep_target(void) {
    if (s_sweep_config.manual_mode) return s_sweep_config.manual_speed;

    static TickType_t last_toggle = 0;
    static bool phase1 = true;
    TickType_t now = xTaskGetTickCount();
    if ((now - last_toggle) >= pdMS_TO_TICKS(s_sweep_config.interval_ms)) {
        phase1 = !phase1;
        last_toggle = now;
    }
    return phase1 ? s_sweep_config.speed1 : s_sweep_config.speed2;
}

static void enter(void) {
    ESP_LOGI(TAG, "Entering CALIBRATION mode");
    s_sweep_config.speed1 = atof(CONFIG_VELOCITY_CTRL_SWEEP_SPEED_1);
    s_sweep_config.speed2 = atof(CONFIG_VELOCITY_CTRL_SWEEP_SPEED_2);
    s_sweep_config.interval_ms = CONFIG_VELOCITY_CTRL_SWEEP_TIME_MS;
    s_sweep_config.manual_mode = false;

    if (cal_telemetry == NULL) {
        cal_telemetry = telemetry_create("robot/telemetry/calibration", "motor_cal", CONFIG_TELEMETRY_INTERVAL_CALIBRATION_MS);
    }
    mqtt_custom_client_register_topic_callback(CALIBRATION_CONFIG_TOPIC, mqtt_cal_callback);
    if (mqtt_custom_client_is_connected()) {
        mqtt_custom_client_subscribe(CALIBRATION_CONFIG_TOPIC, 0);
    }
}

static void execute(motor_driver_mcpwm_t* motors, 
                    motor_velocity_ctrl_handle_t ctrl_left, 
                    motor_velocity_ctrl_handle_t ctrl_right, 
                    float dt_s)
{
    robot_state_context_t* ctx = state_machine_get_context();
    shared_memory_t* shm = shared_memory_get();

    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    float bat_mv = shm->sensors.battery_voltage; 
    float cur_spd_left = shm->sensors.motor_speed_left;
    float cur_spd_right = shm->sensors.motor_speed_right;
    uint8_t motor_mask = shm->calibration_motor_mask;
    xSemaphoreGive(shm->mutex);

    if (bat_mv < 5000.0f) { bat_mv = 16800.0f; }

    float target_left = 0.0f;
    float target_right = 0.0f;

    if (ctx->current_mode == MODE_CALIBRATE_MOTORS) {
        float sweep_target = get_sweep_target();
        if (motor_mask & 0x01) target_left = sweep_target;
        if (motor_mask & 0x02) target_right = sweep_target;
    }

    motor_velocity_input_t input_l = { .target_speed = target_left, .current_speed = cur_spd_left, .battery_mv = bat_mv };
    motor_velocity_input_t input_r = { .target_speed = target_right, .current_speed = cur_spd_right, .battery_mv = bat_mv };

    float raw_pwm_l = 0.0f, raw_pwm_r = 0.0f;
    motor_velocity_diag_t diag_l, diag_r;

    motor_velocity_ctrl_update(ctrl_left,  &input_l, dt_s, &raw_pwm_l, &diag_l);
    motor_velocity_ctrl_update(ctrl_right, &input_r, dt_s, &raw_pwm_r, &diag_r);

    motor_mcpwm_set(motors, (int16_t)(raw_pwm_l * 10.0f), (int16_t)(raw_pwm_r * 10.0f));

    if (cal_telemetry) {
        if (motor_mask & 0x01) {
            telemetry_add_float(cal_telemetry, "target_l",  diag_l.target_ramped);
            telemetry_add_float(cal_telemetry, "actual_l",  cur_spd_left);
            telemetry_add_float(cal_telemetry, "v_ff_l",    diag_l.feed_forward_v);
            telemetry_add_float(cal_telemetry, "v_p_l",     diag_l.p_v);
            telemetry_add_float(cal_telemetry, "v_i_l",     diag_l.i_v);
            telemetry_add_float(cal_telemetry, "v_d_l",     diag_l.d_v);
            telemetry_add_float(cal_telemetry, "v_final_l", diag_l.final_v);
            telemetry_add_float(cal_telemetry, "pwm_l",     diag_l.pwm_duty);
        }
        if (motor_mask & 0x02) {
            telemetry_add_float(cal_telemetry, "target_r",  diag_r.target_ramped);
            telemetry_add_float(cal_telemetry, "actual_r",  cur_spd_right);
            telemetry_add_float(cal_telemetry, "v_ff_r",    diag_r.feed_forward_v);
            telemetry_add_float(cal_telemetry, "v_p_r",     diag_r.p_v);
            telemetry_add_float(cal_telemetry, "v_i_r",     diag_r.i_v);
            telemetry_add_float(cal_telemetry, "v_d_r",     diag_r.d_v);
            telemetry_add_float(cal_telemetry, "v_final_r", diag_r.final_v);
            telemetry_add_float(cal_telemetry, "pwm_r",     diag_r.pwm_duty);
        }
        telemetry_commit_point(cal_telemetry);
    }
}

static void exit_mode(motor_driver_mcpwm_t* motors) {
    ESP_LOGI(TAG, "Exiting CALIBRATION mode");
    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_calibrate = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};
