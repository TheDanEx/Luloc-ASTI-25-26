#include "state_machine.h"
#include "esp_log.h"
#include "mqtt_custom_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "state_machine";

static robot_state_context_t g_state = {
    .current_state = STATE_INIT,
    .previous_state = STATE_INIT,
    .current_mode = MODE_NONE,
    .mqtt_connected = false,
    .mode_requires_mqtt = false,
    .state_time_ms = 0,
    .error_code = 0
};

static TickType_t g_state_start_time = 0;

/**
 * Publish state change event to MQTT
 */
static void publish_state_event(const char *event_msg)
{
    // Always log to serial monitor as requested
    ESP_LOGI(TAG, "STATE EVENT: %s", event_msg);

    if (mqtt_custom_client_is_connected()) {
        int msg_id = mqtt_custom_client_publish("robot/events", event_msg, 0, 1, 0);
        if (msg_id >= 0) {
            ESP_LOGD(TAG, "Published event: %s (msg_id=%d)", event_msg, msg_id);
        }
    } else {
        ESP_LOGW(TAG, "MQTT not connected, event not published: %s", event_msg);
    }
}

void state_machine_init(void)
{
    g_state.current_state = STATE_INIT;
    g_state.current_mode = MODE_NONE;
    g_state.mqtt_connected = false;
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
        
    case CONDITION_MQTT_CONNECTED:
        return ctx->mqtt_connected;
        
    case CONDITION_MQTT_DISCONNECTED:
        return !ctx->mqtt_connected;
        
    case CONDITION_TIMEOUT:
        return ctx->current_state_time_ms >= data_val;
        
    case CONDITION_ALWAYS:
        return true;
        
    default:
        return false;
    }
}

robot_state_t state_machine_update(void)
{
    TickType_t now = xTaskGetTickCount();
    g_state.state_time_ms = (uint32_t)((now - g_state_start_time) / portTICK_PERIOD_MS);
    
    // Update mode requirement
    g_state.mode_requires_mqtt = state_machine_requires_mqtt();
    
    // Prepare context for condition checking
    condition_context_t ctx = {
        .current_state_time_ms = g_state.state_time_ms,
        .mqtt_connected = g_state.mqtt_connected,
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
                
                if (rule->event_name) {
                   publish_state_event(rule->event_name);
                }
                
                // Break after first valid transition to avoid multi-hop in one cycle
                break;
            }
        }
    }
    
    return g_state.current_state;
}

void state_machine_notify_mqtt_status(bool connected)
{
    bool was_connected = g_state.mqtt_connected;
    g_state.mqtt_connected = connected;
    
    if (connected != was_connected) {
        if (connected) {
            ESP_LOGI(TAG, "[EVENT] MQTT CONNECTED - state: %s, mode: %d",
                     get_state_name(g_state.current_state),
                     g_state.current_mode);
        } else {
            ESP_LOGW(TAG, "[EVENT] MQTT DISCONNECTED - state: %s, mode: %d",
                     get_state_name(g_state.current_state),
                     g_state.current_mode);
        }
    }
}

bool state_machine_request_mode(robot_mode_t new_mode)
{
    const mode_config_t *config = get_mode_config(new_mode);
    if (!config) {
        ESP_LOGE(TAG, "Unknown mode requested: %d", new_mode);
        return false;
    }

    // Check MQTT requirement
    if (config->requires_mqtt && !g_state.mqtt_connected) {
        ESP_LOGW(TAG, "Mode %s requires MQTT but not connected", get_mode_name(new_mode));
        return false;
    }
    
    if (config->requires_mqtt && g_state.current_state == STATE_MQTT_LOST_ERROR) {
        ESP_LOGE(TAG, "Cannot switch to MQTT-dependent mode in ERROR state");
        return false;
    }
    
    ESP_LOGI(TAG, "Mode transition: %d → %d", g_state.current_mode, new_mode);
    g_state.current_mode = new_mode;
    return true;
}

robot_state_context_t* state_machine_get_context(void)
{
    return &g_state;
}

bool state_machine_is_autonomous_safe(void)
{
    // Safe to operate autonomously if:
    // 1. Not in ERROR state
    // AND has a valid mode
    
    return g_state.current_state != STATE_MQTT_LOST_ERROR &&
           g_state.current_mode != MODE_NONE;
}

bool state_machine_requires_mqtt(void)
{
    const mode_config_t *config = get_mode_config(g_state.current_mode);
    return config ? config->requires_mqtt : false;
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
