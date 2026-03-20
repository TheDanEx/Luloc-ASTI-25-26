#ifndef MAIN_MODES_MODES_H
#define MAIN_MODES_MODES_H

#include "motor.h"
#include "motor_velocity_ctrl.h"
#include "state_machine.h"

/**
 * @brief Initialize all mode modules.
 */
void modes_init(void);

/**
 * @brief Global mode dispatcher.
 * 
 * Detects mode changes, handles transitions (enter/exit), and executes current logic.
 */
void modes_execute(motor_driver_mcpwm_t* motors, 
                   motor_velocity_ctrl_handle_t ctrl_left, 
                   motor_velocity_ctrl_handle_t ctrl_right, 
                   float dt_s);

#endif // MAIN_MODES_MODES_H
