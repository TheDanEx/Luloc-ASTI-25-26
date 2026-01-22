#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * MQTT Watchdog - Monitors connection health
 * Automatically recovers from disconnections
 * Triggers safety shutdown if connection lost > threshold
 */

typedef enum {
    MQTT_HEALTH_CONNECTED,        // Normal operation
    MQTT_HEALTH_DISCONNECTED,     // Lost connection, attempting recovery
    MQTT_HEALTH_FAILED,           // Connection failed for too long - SAFETY STOP
} mqtt_health_status_t;

/**
 * Initialize MQTT watchdog monitoring
 */
void mqtt_watchdog_init(void);

/**
 * Check connection health (call periodically)
 */
mqtt_health_status_t mqtt_watchdog_check(void);

/**
 * Get current connection status
 */
bool mqtt_watchdog_is_connected(void);

/**
 * Get time disconnected in milliseconds
 */
uint32_t mqtt_watchdog_get_disconnected_time_ms(void);

/**
 * Manual reconnection attempt
 */
bool mqtt_watchdog_reconnect(void);

/**
 * Enable/disable watchdog
 */
void mqtt_watchdog_set_enabled(bool enabled);
