#include "mode_interface.h"
#include "esp_log.h"
#include "motor.h"
#include "shared_memory.h"
#include <math.h>

static const char *TAG = "MODE_SUMO";

// Configurable parameters
#define SUMO_DETECTION_DIST_M 1.5f
#define SUMO_ATTACK_SPEED     0.8f
#define SUMO_SEARCH_SPEED     0.4f

static void enter(void) {
    ESP_LOGI(TAG, "Entering SUMO mode - PROTECT THE RING!");
}

static void exit_mode(motor_driver_mcpwm_t* motors) {
    ESP_LOGI(TAG, "Exiting SUMO mode - At ease.");
    motor_mcpwm_set(motors, 0, 0);
}

static void execute(motor_driver_mcpwm_t* motors, 
                   motor_velocity_ctrl_handle_t ctrl_left, 
                   motor_velocity_ctrl_handle_t ctrl_right, 
                   float dt_s) 
{
    shared_memory_t* shm = shared_memory_get();
    if (shm == NULL) return;

    float min_dist_front = 10.0f;
    int best_angle = -1;

    // 1. Scan Frontal Area (-30 to +30 degrees)
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        for (int i = -30; i <= 30; i++) {
            int idx = (i + 360) % 360;
            float d = shm->lidar.distances_m[idx];
            if (d > 0.05f && d < min_dist_front) {
                min_dist_front = d;
                best_angle = i;
            }
        }
        xSemaphoreGive(shm->mutex);
    }

    // 2. State Machine: Search vs Attack
    if (min_dist_front < SUMO_DETECTION_DIST_M) {
        // Opponent detected!
        ESP_LOGI(TAG, "Target found at %.2fm, angle %d! ATTACK!", min_dist_front, best_angle);
        
        // Simple proportional steering to center the target
        float steering = best_angle * 0.01f; 
        float speed_l = SUMO_ATTACK_SPEED * (1.0f + steering);
        float speed_r = SUMO_ATTACK_SPEED * (1.0f - steering);
        
        motor_velocity_input_t in_l = { .target_speed = speed_l, .current_speed = shm->sensors.motor_speed_left, .battery_mv = 16000 };
        motor_velocity_input_t in_r = { .target_speed = speed_r, .current_speed = shm->sensors.motor_speed_right, .battery_mv = 16000 };
        
        float pwm_l, pwm_r;
        motor_velocity_ctrl_update(ctrl_left,  &in_l, dt_s, &pwm_l, NULL);
        motor_velocity_ctrl_update(ctrl_right, &in_r, dt_s, &pwm_r, NULL);
        motor_mcpwm_set(motors, (int16_t)(pwm_l * 10.0f), (int16_t)(pwm_r * 10.0f));

    } else {
        // Search: Spin slowly
        float pwm_l = 15.0f; // 15% power
        float pwm_r = -15.0f;
        motor_mcpwm_set(motors, (int16_t)pwm_l, (int16_t)pwm_r);
    }
}

const mode_interface_t mode_sumo = {
    .enter = enter,
    .exit = exit_mode,
    .execute = execute
};
