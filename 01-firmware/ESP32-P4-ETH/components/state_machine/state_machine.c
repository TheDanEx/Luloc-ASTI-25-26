#include "state_machine.h"
#include "esp_log.h"
#include "shared_memory.h"
#include "mqtt_custom_client.h"
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
 * Determine if mode requires MQTT connection
 */
static bool mode_requires_mqtt(robot_mode_t mode)
{
    switch (mode) {
    case MODE_AUTONOMOUS_PATH:
    case MODE_AUTONOMOUS_OBSTACLE:
        return false;  // Can run without MQTT
        
    case MODE_REMOTE_DRIVE:
    case MODE_TELEMETRY_STREAM:
        return true;   // Requires MQTT for commands/telemetry
        
    case MODE_NONE:
    default:
        return false;
    }
}

robot_state_t state_machine_update(void)
{
    TickType_t now = xTaskGetTickCount();
    g_state.state_time_ms = (uint32_t)((now - g_state_start_time) / portTICK_PERIOD_MS);
    
    // Update mode requirement
    g_state.mode_requires_mqtt = mode_requires_mqtt(g_state.current_mode);
    
    // State machine transitions
    switch (g_state.current_state) {
    case STATE_INIT:
        // Wait for configuration and MQTT connection
        if (g_state.mqtt_connected) {
            ESP_LOGI(TAG, "MQTT connected - transitioning to REMOTE_CONTROLLED");
            g_state.previous_state = STATE_INIT;
            g_state.current_state = STATE_REMOTE_CONTROLLED;
            g_state.current_mode = MODE_REMOTE_DRIVE;
            g_state_start_time = now;
            publish_state_event("STATE_REMOTE_CONTROLLED");
        }
        break;
        
    case STATE_AUTONOMOUS:
        // Running autonomous - doesn't need MQTT, but monitor for changes
        if (g_state.mqtt_connected && g_state.current_mode == MODE_NONE) {
            ESP_LOGI(TAG, "MQTT available - transitioning to TELEMETRY_ONLY");
            g_state.previous_state = STATE_AUTONOMOUS;
            g_state.current_state = STATE_TELEMETRY_ONLY;
            g_state_start_time = now;
            publish_state_event("STATE_TELEMETRY_ONLY");
        }
        break;
        
    case STATE_REMOTE_CONTROLLED:
        // Waiting for commands from MQTT
        if (!g_state.mqtt_connected) {
            ESP_LOGE(TAG, "CRITICAL: MQTT lost during REMOTE_CONTROLLED mode - STOPPING");
            g_state.previous_state = STATE_REMOTE_CONTROLLED;
            g_state.current_state = STATE_MQTT_LOST_ERROR;
            g_state.error_code = 1;  // MQTT lost
            g_state_start_time = now;
            publish_state_event("STATE_MQTT_LOST_ERROR - MQTT_DISCONNECTED");
        }
        break;
        
    case STATE_TELEMETRY_ONLY:
        // Autonomous with telemetry
        if (!g_state.mqtt_connected) {
            ESP_LOGE(TAG, "CRITICAL: MQTT lost during TELEMETRY_ONLY mode - STOPPING");
            g_state.previous_state = STATE_TELEMETRY_ONLY;
            g_state.current_state = STATE_MQTT_LOST_ERROR;
            g_state.error_code = 2;  // MQTT lost during telemetry
            g_state_start_time = now;
            publish_state_event("STATE_MQTT_LOST_ERROR - TELEMETRY_FAILED");
        }
        break;
        
    case STATE_MQTT_LOST_ERROR:
        // MQTT lost - require manual intervention or timeout
        // Try to recover if MQTT comes back
        if (g_state.mqtt_connected && g_state.state_time_ms > 5000) {
            ESP_LOGI(TAG, "MQTT recovered after error state - attempting resume");
            g_state.previous_state = STATE_MQTT_LOST_ERROR;
            g_state.current_state = STATE_REMOTE_CONTROLLED;
            g_state.error_code = 0;
            g_state_start_time = now;
            publish_state_event("STATE_REMOTE_CONTROLLED - RECOVERED");
        }
        break;
        
    case STATE_SHUTDOWN:
        // Graceful shutdown - don't transition
        break;
        
    default:
        ESP_LOGW(TAG, "Unknown state: %d", g_state.current_state);
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
                     state_machine_get_state_name(g_state.current_state),
                     g_state.current_mode);
        } else {
            ESP_LOGW(TAG, "[EVENT] MQTT DISCONNECTED - state: %s, mode: %d",
                     state_machine_get_state_name(g_state.current_state),
                     g_state.current_mode);
        }
    }
}

bool state_machine_request_mode(robot_mode_t new_mode)
{
    // Check if mode is compatible with current state
    bool requires_mqtt = mode_requires_mqtt(new_mode);
    
    if (requires_mqtt && !g_state.mqtt_connected) {
        ESP_LOGW(TAG, "Mode %d requires MQTT but not connected", new_mode);
        return false;
    }
    
    if (requires_mqtt && g_state.current_state == STATE_MQTT_LOST_ERROR) {
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
    // 1. In AUTONOMOUS state, OR
    // 2. In TELEMETRY_ONLY with MQTT connected, OR
    // 3. Not in ERROR state
    
    return g_state.current_state != STATE_MQTT_LOST_ERROR &&
           g_state.current_mode != MODE_NONE;
}

bool state_machine_requires_mqtt(void)
{
    return g_state.mode_requires_mqtt;
}

const char* state_machine_get_state_name(robot_state_t state)
{
    static const char *names[] = {
        "INIT",
        "AUTONOMOUS",
        "REMOTE_CONTROLLED",
        "TELEMETRY_ONLY",
        "MQTT_LOST_ERROR",
        "SHUTDOWN"
    };
    
    if (state < sizeof(names) / sizeof(names[0])) {
        return names[state];
    }
    return "UNKNOWN";
}
