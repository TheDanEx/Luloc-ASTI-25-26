#include "mode_interface.h"
#include "esp_log.h"
#include "shared_memory.h"
#include "mqtt_custom_client.h"
#include "cJSON.h"
#include "motor.h"
#include "esp_timer.h"

static const char *TAG = "MODE_TELEOP";

#define TELEOP_TOPIC "robot/teleop"

static void mqtt_teleop_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (data == NULL || data_len <= 0) return;
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) return;

    cJSON *l = cJSON_GetObjectItem(root, "l");
    cJSON *r = cJSON_GetObjectItem(root, "r");

    shared_memory_t* shm = shared_memory_get();
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (l) shm->teleop.target_speed_left = l->valuedouble;
        if (r) shm->teleop.target_speed_right = r->valuedouble;
        shm->teleop.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
        xSemaphoreGive(shm->mutex);
    }

    cJSON_Delete(root);
}

static void enter(void) {
    ESP_LOGI(TAG, "Entering TELEOPERATION mode");
    
    // Initialize targets to 0
    shared_memory_t* shm = shared_memory_get();
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        shm->teleop.target_speed_left = 0;
        shm->teleop.target_speed_right = 0;
        xSemaphoreGive(shm->mutex);
    }

    mqtt_custom_client_register_topic_callback(TELEOP_TOPIC, mqtt_teleop_callback);
    if (mqtt_custom_client_is_connected()) {
        mqtt_custom_client_subscribe(TELEOP_TOPIC, 0);
    }
}

static void execute(motor_driver_mcpwm_t* motors, 
                    motor_velocity_ctrl_handle_t ctrl_left, 
                    motor_velocity_ctrl_handle_t ctrl_right, 
                    float dt_s) 
{
    shared_memory_t* shm = shared_memory_get();
    
    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    float target_l = shm->teleop.target_speed_left;
    float target_r = shm->teleop.target_speed_right;
    float bat_mv   = shm->sensors.battery_voltage;
    float cur_l    = shm->sensors.motor_speed_left;
    float cur_r    = shm->sensors.motor_speed_right;
    xSemaphoreGive(shm->mutex);

    // Fallback battery
    if (bat_mv < 5000) bat_mv = 16800;

    motor_velocity_input_t input_l = { .target_speed = target_l, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t input_r = { .target_speed = target_r, .current_speed = cur_r, .battery_mv = bat_mv };

    float pwm_l, pwm_r;
    motor_velocity_ctrl_update(ctrl_left,  &input_l, dt_s, &pwm_l, NULL);
    motor_velocity_ctrl_update(ctrl_right, &input_r, dt_s, &pwm_r, NULL);

    motor_mcpwm_set(motors, (int16_t)(pwm_l * 10.0f), (int16_t)(pwm_r * 10.0f));
}

static void exit_mode(motor_driver_mcpwm_t* motors) {
    ESP_LOGI(TAG, "Exiting TELEOPERATION mode");
    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_teleoperation = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};
