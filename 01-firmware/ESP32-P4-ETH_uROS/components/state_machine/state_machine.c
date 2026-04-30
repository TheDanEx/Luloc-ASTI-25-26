#include "state_machine.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdio.h>

// =============================================================================
// Definitions & Local State
// =============================================================================

static const char *TAG = "state_machine";

static robot_state_context_t g_state = {
    .current_state = STATE_INIT,
    .previous_state = STATE_INIT,
    .current_mode = MODE_NONE,
    .state_time_ms = 0,
    .error_code = 0
};

static TickType_t g_state_start_time = 0;

// =============================================================================
// Internal Handlers
// =============================================================================



// =============================================================================
// Public API: Lifecycle
// =============================================================================

void state_machine_init(void)
{
    g_state.current_state = STATE_INIT;
    g_state.current_mode = MODE_NONE;
    g_state_start_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "State machine initialized");
}



/**
 * Check if a specific condition is met
 */
static bool check_condition(transition_condition_t condition, uint32_t data_val, const condition_context_t *ctx)
{
    switch (condition) {
    case CONDITION_NONE:
        return true;
        
    case CONDITION_TIMEOUT:
        return ctx->current_state_time_ms >= data_val;
        
    case CONDITION_ALWAYS:
        return true;
        
    default:
        return false;
    }
}

// =============================================================================
// Public API: State Control
// =============================================================================

robot_state_t state_machine_update(void)
{
    TickType_t now = xTaskGetTickCount();
    g_state.state_time_ms = (uint32_t)((now - g_state_start_time) / portTICK_PERIOD_MS);

    // Prepare context for condition checking
    condition_context_t ctx = {
        .current_state_time_ms = g_state.state_time_ms,
        .current_mode = g_state.current_mode
    };
    
    // Iterate through transition table
    for (size_t i = 0; i < transition_table_size; i++) {
        const state_transition_rule_t *rule = &transition_table[i];
        
        // Match current state
        if (rule->from_state == g_state.current_state) {
            
            // Check condition
            if (check_condition(rule->condition, rule->data_val, &ctx)) {
                
                // execute transition
                ESP_LOGI(TAG, "Transition: %s -> %s (Event: %s)",
                         get_state_name(g_state.current_state),
                         get_state_name(rule->to_state),
                         rule->event_name ? rule->event_name : "NONE");
                         
                g_state.previous_state = g_state.current_state;
                g_state.current_state = rule->to_state;
                g_state_start_time = now;
                
                // Optional mode change
                if (rule->new_mode != MODE_NONE) {
                    g_state.current_mode = rule->new_mode;
                    ESP_LOGI(TAG, "Auto Mode Switch: %s", get_mode_name(rule->new_mode));
                }
                
                // Break after first valid transition to avoid multi-hop in one cycle
                break;
            }
        }
    }
    
    return g_state.current_state;
}



bool state_machine_request_mode(robot_mode_t new_mode, bool force)
{
    const mode_config_t *config = get_mode_config(new_mode);
    if (!config) {
        ESP_LOGE(TAG, "Unknown mode requested: %d", new_mode);
        return false;
    }

    if (force) {
        ESP_LOGW(TAG, "FORCING Mode %s regardless of conditions", get_mode_name(new_mode));
    }

    ESP_LOGI(TAG, "Mode transition: %d -> %d", g_state.current_mode, new_mode);
    g_state.current_mode = new_mode;
    return true;
}

// =============================================================================
// Public API: Utilities & Getters
// =============================================================================

robot_state_context_t* state_machine_get_context(void)
{
    return &g_state;
}

bool state_machine_is_autonomous_safe(void)
{
    // Safe to operate autonomously if:
    // 1. Not in ERROR state
    // AND has a valid mode
    
    return g_state.current_mode != MODE_NONE;
}

uint32_t state_machine_get_active_sensors(void)
{
    const mode_config_t *config = get_mode_config(g_state.current_mode);
    return config ? config->sensor_mask : 0;
}

const char* state_machine_get_state_name(robot_state_t state)
{
    return get_state_name(state);
}
