#include "mode_calibrate.h"
#include "esp_log.h"
#include "shared_memory.h"
#include "telemetry_manager.h"
#include <stdlib.h>

static telemetry_handle_t cal_telemetry = NULL;

void mode_calibrate_execute(motor_driver_mcpwm_t* motors, 
                            motor_velocity_ctrl_handle_t ctrl_left, 
                            motor_velocity_ctrl_handle_t ctrl_right, 
                            float dt_s)
{
    robot_state_context_t* ctx = state_machine_get_context();
    shared_memory_t* shm = shared_memory_get();

    if (cal_telemetry == NULL) {
        cal_telemetry = telemetry_create("robot/telemetry/calibration", "motor_cal", 100);
    }

    // Safely snapshot shared variables needed for the iteration
    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    float bat_mv = shm->sensors.battery_voltage; 
    float cur_spd_left = shm->sensors.motor_speed_left;
    float cur_spd_right = shm->sensors.motor_speed_right;
    uint8_t motor_mask = shm->calibration_motor_mask;
    xSemaphoreGive(shm->mutex);

    // Provide a safe fallback for battery calculation if undefined
    if (bat_mv < 5000.0f) { bat_mv = 16800.0f; }

    float target_left = 0.0f;
    float target_right = 0.0f;

    // Route logic based on the specific type of calibration requested
    switch (ctx->current_mode) {
        
        case MODE_CALIBRATE_MOTORS: {
            // Velocity Sweeper Strategy
            float sweep_target = motor_velocity_ctrl_get_sweep_target();
            if (motor_mask & 0x01) target_left = sweep_target;
            if (motor_mask & 0x02) target_right = sweep_target;
            break;
        }

        case MODE_CALIBRATE_LINE: {
            // Line Follower Camera/PID tuning strategy
            target_left = 0.0f;
            target_right = 0.0f;
            break;
        }

        default:
            // Fail-safe
            target_left = 0.0f;
            target_right = 0.0f;
            break;
    }

    // Prepare inputs
    motor_velocity_input_t input_l = { .target_speed = target_left, .current_speed = cur_spd_left, .battery_mv = bat_mv };
    motor_velocity_input_t input_r = { .target_speed = target_right, .current_speed = cur_spd_right, .battery_mv = bat_mv };

    float raw_pwm_l = 0.0f;
    float raw_pwm_r = 0.0f;

    // Calculate closed-loop PID response
    motor_velocity_ctrl_update(ctrl_left,  &input_l, dt_s, &raw_pwm_l);
    motor_velocity_ctrl_update(ctrl_right, &input_r, dt_s, &raw_pwm_r);



    // Push scaled duty cycle directly to hardware (Percentage -100 to 100 -> Promille -1000 to 1000)
    motor_mcpwm_set(motors, (int16_t)(raw_pwm_l * 10.0f), (int16_t)(raw_pwm_r * 10.0f));

    // Telemetry for Live PID Tuning Comparison
    if (cal_telemetry) {
        telemetry_add_float(cal_telemetry, "target_l", target_left);
        telemetry_add_float(cal_telemetry, "actual_l", cur_spd_left);
        telemetry_add_float(cal_telemetry, "target_r", target_right);
        telemetry_add_float(cal_telemetry, "actual_r", cur_spd_right);
        telemetry_commit_point(cal_telemetry);
    }
}
