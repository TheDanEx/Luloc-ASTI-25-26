# MQTT Custom Client Component

Simplified wrapper around ESP-IDF MQTT component with a clean public API.

## Features

- ✅ Automatic connection to MQTT broker
- ✅ Simple publish/subscribe API
- ✅ Event logging and error handling
- ✅ Connection state tracking
- ✅ Configurable broker URL via menuconfig
- ✅ Automatic startup event publication on connection

## Usage

### Basic initialization

```c
#include "mqtt_custom_client.h"

void app_main(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    // Your network initialization here (Ethernet, WiFi, etc.)

    // Initialize MQTT
    mqtt_custom_client_init();
}
```

### Publish a message

```c
// Publish to robot/events with QoS 1
mqtt_custom_client_publish("robot/events", "Hello MQTT", 0, 1, 0);
```

### Subscribe to a topic

```c
// Subscribe to robot/cmd with QoS 1
mqtt_custom_client_subscribe("robot/cmd", 1);

// Messages will be logged when received (see mqtt_event_handler)
```

### Check connection status

```c
if (mqtt_custom_client_is_connected()) {
    ESP_LOGI(TAG, "MQTT is connected!");
}
```

## Configuration

Configure via `menuconfig`:

```
Component config → MQTT Custom Client Configuration → Broker URL
```

Default: `mqtt://192.168.42.15:51111`

## Dependencies

- `mqtt` - ESP-IDF MQTT component
- `esp_event` - Event loop system
- `esp_netif` - Networking interface
- `nvs_flash` - Non-volatile storage

## Event Handling

The component automatically handles all MQTT events internally:

- **MQTT_EVENT_CONNECTED**: Logs connection, publishes startup event
- **MQTT_EVENT_DISCONNECTED**: Updates connection state
- **MQTT_EVENT_DATA**: Logs received messages
- **MQTT_EVENT_ERROR**: Logs errors with details

Events are processed by the internal `mqtt_event_handler()` - no additional setup needed.

## API Reference

### esp_err_t mqtt_custom_client_init(void)

Initialize and start the MQTT client. Must be called after event loop is created.

**Returns:** ESP_OK on success, ESP_FAIL on failure

### int mqtt_custom_client_publish(const char *topic, const char *data, int len, int qos, int retain)

Publish a message to a topic.

**Parameters:**

- `topic`: Topic name
- `data`: Message data
- `len`: Data length (0 for NULL-terminated string)
- `qos`: Quality of Service (0 or 1)
- `retain`: Retain flag (0 or 1)

**Returns:** Message ID on success, -1 on failure

### int mqtt_custom_client_subscribe(const char \*topic, int qos)

Subscribe to a topic.

**Parameters:**

- `topic`: Topic name
- `qos`: Quality of Service (0 or 1)

**Returns:** Message ID on success, -1 on failure

### int mqtt_custom_client_unsubscribe(const char \*topic)

Unsubscribe from a topic.

**Parameters:**

- `topic`: Topic name

**Returns:** Message ID on success, -1 on failure

### bool mqtt_custom_client_is_connected(void)

Check if MQTT client is currently connected.

**Returns:** true if connected, false otherwise
