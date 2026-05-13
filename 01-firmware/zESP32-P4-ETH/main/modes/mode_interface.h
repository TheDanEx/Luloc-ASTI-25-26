#ifndef MAIN_MODES_MODE_INTERFACE_H
#define MAIN_MODES_MODE_INTERFACE_H

#include "motor.h"
#include "motor_velocity_ctrl.h"

/**
 * @brief Standard Mode Interface
 * 
 * Defines the lifecycle of any operational mode.
 */
typedef struct {
    /**
     * @brief Called once when the mode is entered.
     */
    void (*enter)(void);

    /**
     * @brief Called periodically (100Hz) inside the control loop.
     */
    void (*execute)(motor_driver_mcpwm_t* motors, 
                    motor_velocity_ctrl_handle_t ctrl_left, 
                    motor_velocity_ctrl_handle_t ctrl_right, 
                    float dt_s);

    /**
     * @brief Called once when the mode is exited.
     */
    void (*exit)(motor_driver_mcpwm_t* motors);
} mode_interface_t;

#endif // MAIN_MODES_MODE_INTERFACE_H
