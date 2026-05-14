#include "mode_interface.h"
#include "esp_log.h"
#include "motor.h"
#include "shared_memory.h"
#include <math.h>

static const char *TAG = "MODE_SUMO";

// Configurable parameters (0.0 to 1.0 for velocity control, or 0-1000 for PWM)
#define SUMO_DETECTION_DIST_M 1.2f
#define SUMO_ATTACK_SPEED     0.4f  // m/s (Much slower as requested)
#define SUMO_SEARCH_PWM       120   // 12% power for spinning in place
#define SUMO_DETECTION_ARC    40    // +/- 40 degrees

static void enter(void) {
    ESP_LOGI(TAG, "SUMO: Entering ring. Target distance: %.1fm", SUMO_DETECTION_DIST_M);
}

static void exit_mode(motor_driver_mcpwm_t* motors) {
    ESP_LOGI(TAG, "SUMO: Stopping motors.");
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

    // 1. Scan Frontal Area
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        for (int i = -SUMO_DETECTION_ARC; i <= SUMO_DETECTION_ARC; i++) {
            int idx = (i + 360) % 360;
            float d = shm->lidar.distances_m[idx];
            // Filter 0 (no data) and too close (self-collision/noise)
            if (d > 0.10f && d < min_dist_front) {
                min_dist_front = d;
                best_angle = i;
            }
        }
        xSemaphoreGive(shm->mutex);
    }

    // 2. Behavior Logic
    if (min_dist_front < SUMO_DETECTION_DIST_M) {
        // --- ENEMY DETECTED ---
        ESP_LOGI(TAG, "!!! ENEMY DETECTED at %.2fm (Angle: %d) !!!", min_dist_front, best_angle);
        
        // Steering proportional to angle
        float steering = best_angle * 0.005f; 
        float speed_l = SUMO_ATTACK_SPEED * (1.0f + steering);
        float speed_r = SUMO_ATTACK_SPEED * (1.0f - steering);
        
        // Closed-loop velocity control for stable attack
        motor_velocity_input_t in_l = { .target_speed = speed_l, .current_speed = shm->sensors.motor_speed_left, .battery_mv = 16000 };
        motor_velocity_input_t in_r = { .target_speed = speed_r, .current_speed = shm->sensors.motor_speed_right, .battery_mv = 16000 };
        
        float pwm_l_f, pwm_r_f;
        motor_velocity_ctrl_update(ctrl_left,  &in_l, dt_s, &pwm_l_f, NULL);
        motor_velocity_ctrl_update(ctrl_right, &in_r, dt_s, &pwm_r_f, NULL);
        
        // Convert -1.0..1.0 to -1000..1000
        motor_mcpwm_set(motors, (int16_t)(pwm_l_f * 1000.0f), (int16_t)(pwm_r_f * 1000.0f));

    } else {
        // --- SEARCHING ---
        // Spin in place: Symmetric opposite PWM
        // Positive L, Negative R = Turn Right
        motor_mcpwm_set(motors, SUMO_SEARCH_PWM, -SUMO_SEARCH_PWM);
    }
}

const mode_interface_t mode_sumo = {
    .enter = enter,
    .exit = exit_mode,
    .execute = execute
};
