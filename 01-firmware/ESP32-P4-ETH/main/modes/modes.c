#include "modes.h"
#include "mode_interface.h"
#include "esp_log.h"
#include <stddef.h>

static const char *TAG = "MODES_DISPATCHER";

// Extern declarations of mode interfaces
extern const mode_interface_t mode_idle;
extern const mode_interface_t mode_calibrate;
extern const mode_interface_t mode_teleoperation;
extern const mode_interface_t mode_calibrate_line;
extern const mode_interface_t mode_follow_line;

// Local state
static robot_mode_t s_active_mode = MODE_NONE;
static const mode_interface_t* s_active_interface = &mode_idle;

/**
 * Helper to get the interface for a specific mode
 */
static const mode_interface_t* get_interface_for_mode(robot_mode_t mode) {
    switch (mode) {
        case MODE_NONE:               return &mode_idle;
        case MODE_CALIBRATE_MOTORS:   return &mode_calibrate;
        case MODE_REMOTE_DRIVE:       return &mode_teleoperation;
        case MODE_CALIBRATE_LINE:     return &mode_calibrate_line;
        case MODE_FOLLOW_LINE:        return &mode_follow_line;
        default:                      return &mode_idle;
    }
}

void modes_init(void) {
    ESP_LOGI(TAG, "Initializing modes system");
    s_active_mode = MODE_NONE;
    s_active_interface = &mode_idle;
    
    if (s_active_interface->enter) {
        s_active_interface->enter();
    }
}

void modes_execute(motor_driver_mcpwm_t* motors, 
                   motor_velocity_ctrl_handle_t ctrl_left, 
                   motor_velocity_ctrl_handle_t ctrl_right, 
                   float dt_s) 
{
    robot_state_context_t* ctx = state_machine_get_context();
    robot_mode_t target_mode = ctx->current_mode;

    // Handle Mode Transition
    if (target_mode != s_active_mode) {
        ESP_LOGW(TAG, "Transitioning: %s -> %s", get_mode_name(s_active_mode), get_mode_name(target_mode));
        
        // 1. Exit previous mode
        if (s_active_interface && s_active_interface->exit) {
            s_active_interface->exit(motors);
        }

        // 2. Switch interface
        s_active_mode = target_mode;
        s_active_interface = get_interface_for_mode(target_mode);

        // 3. Enter new mode
        if (s_active_interface && s_active_interface->enter) {
            s_active_interface->enter();
        }
    }

    // Execute current mode logic
    if (s_active_interface && s_active_interface->execute) {
        s_active_interface->execute(motors, ctrl_left, ctrl_right, dt_s);
    }
}
