#include "mode_interface.h"
#include "esp_log.h"
#include "motor.h"

static const char *TAG = "MODE_IDLE";

static void enter(void) {
    ESP_LOGI(TAG, "Entering IDLE mode");
}

static void execute(motor_driver_mcpwm_t* motors, 
                    motor_velocity_ctrl_handle_t ctrl_left, 
                    motor_velocity_ctrl_handle_t ctrl_right, 
                    float dt_s) 
{
    motor_mcpwm_stop(motors);
}

static void exit_mode(motor_driver_mcpwm_t* motors) {
    ESP_LOGI(TAG, "Exiting IDLE mode");
}

const mode_interface_t mode_idle = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};
