#include "mode_interface.h"
#include "esp_log.h"
#include "shared_memory.h"
#include "motor.h"
#include "esp_timer.h"

static const char *TAG = "MODE_TELEOP";


static void enter(void) {
    ESP_LOGI(TAG, "Entering TELEOPERATION mode");
    
    // Initialize targets to 0
    shared_memory_t* shm = shared_memory_get();
    if (shm == NULL) {
        ESP_LOGE(TAG, "Shared memory unavailable on teleop enter");
        return;
    }

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        shm->teleop.target_speed_left = 0;
        shm->teleop.target_speed_right = 0;
        xSemaphoreGive(shm->mutex);
    }

}

static void execute(motor_driver_mcpwm_t* motors, 
                    motor_velocity_ctrl_handle_t ctrl_left, 
                    motor_velocity_ctrl_handle_t ctrl_right, 
                    float dt_s) 
{
    shared_memory_t* shm = shared_memory_get();
    if (shm == NULL) {
        motor_mcpwm_stop(motors);
        return;
    }
    
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        motor_mcpwm_stop(motors);
        return;
    }

    float target_l = shm->teleop.target_speed_left;
    float target_r = shm->teleop.target_speed_right;
    float bat_mv   = shm->sensors.battery_voltage;
    uint32_t last_update_ms = shm->teleop.last_update_ms;
    float cur_l    = shm->sensors.motor_speed_left;
    float cur_r    = shm->sensors.motor_speed_right;
    xSemaphoreGive(shm->mutex);

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if ((now_ms - last_update_ms) > 500) {
        target_l = 0.0f;
        target_r = 0.0f;
    }
    // Fallback battery
    if (bat_mv < 5000) bat_mv = 16800;

    motor_velocity_input_t input_l = { .target_speed = target_l, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t input_r = { .target_speed = target_r, .current_speed = cur_r, .battery_mv = bat_mv };

    float pwm_l = 0.0f;
    float pwm_r = 0.0f;
    esp_err_t left_err = motor_velocity_ctrl_update(ctrl_left, &input_l, dt_s, &pwm_l, NULL);
    esp_err_t right_err = motor_velocity_ctrl_update(ctrl_right, &input_r, dt_s, &pwm_r, NULL);
    if (left_err != ESP_OK || right_err != ESP_OK) {
        motor_mcpwm_stop(motors);
        return;
    }

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
