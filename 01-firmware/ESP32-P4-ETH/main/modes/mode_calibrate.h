#pragma once

#include "motor.h"
#include "motor_velocity_ctrl.h"
#include "state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Main execution function for all calibration sub-modes.
 * 
 * Called periodically (e.g., at 100Hz) from the real-time control task 
 * when the state machine is in one of the MODE_CALIBRATE_* states.
 * 
 * @param motors Pointer to the initialized motor driver structure.
 * @param ctrl_left Handle to the left motor velocity PID controller.
 * @param ctrl_right Handle to the right motor velocity PID controller.
 * @param dt_s Delta time in seconds since the last call.
 */
void mode_calibrate_execute(motor_driver_mcpwm_t* motors, 
                            motor_velocity_ctrl_handle_t ctrl_left, 
                            motor_velocity_ctrl_handle_t ctrl_right, 
                            float dt_s);

#ifdef __cplusplus
}
#endif
