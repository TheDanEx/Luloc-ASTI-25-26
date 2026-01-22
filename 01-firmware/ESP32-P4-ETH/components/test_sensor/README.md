# Test Sensor Component

Simple test sensor component that simulates a sensor by tracking ESP32 uptime.

## Purpose

- Acts as a dummy sensor for testing/development
- Returns the time elapsed since ESP32 boot
- Useful for testing task_monitor, task_comms, and MQTT integration

## Features

- ✅ Simple initialization
- ✅ Multiple uptime units (ms, s, minutes)
- ✅ Formatted string output (e.g., "1h 23m 45s 123ms")
- ✅ No external dependencies (uses only esp_timer)

## Usage

### Basic usage

```c
#include "test_sensor.h"

void my_task(void *arg) {
    test_sensor_init();

    while(1) {
        uint32_t uptime_sec = test_sensor_get_uptime_sec();
        printf("ESP32 uptime: %u seconds\n", uptime_sec);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### Get structured data

```c
test_sensor_data_t data;
test_sensor_read(&data);

printf("Uptime: %lu ms\n", data.uptime_ms);
printf("Uptime: %lu seconds\n", data.uptime_sec);
printf("Uptime: %lu minutes\n", data.uptime_min);
```

### Get formatted string

```c
char uptime_str[32];
test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
printf("Uptime: %s\n", uptime_str);  // Output: "0h 0m 5s 234ms"
```

## API Reference

### esp_err_t test_sensor_init(void)

Initialize the test sensor. Must be called before reading data.

**Returns:** ESP_OK on success

### esp_err_t test_sensor_read(test_sensor_data_t \*data)

Get structured uptime data.

**Parameters:**

- `data`: Pointer to structure to fill with uptime data

**Returns:** ESP_OK on success, ESP_ERR_INVALID_ARG if data is NULL

### uint32_t test_sensor_get_uptime_ms(void)

Get uptime in milliseconds.

**Returns:** Uptime in milliseconds

### uint32_t test_sensor_get_uptime_sec(void)

Get uptime in seconds.

**Returns:** Uptime in seconds

### const char *test_sensor_get_uptime_str(char *buffer, size_t buffer_size)

Get uptime as formatted string.

**Parameters:**

- `buffer`: Buffer to store formatted string
- `buffer_size`: Size of buffer (minimum 16 bytes recommended)

**Returns:** Pointer to buffer

## Example: Using with task_monitor

```c
#include "test_sensor.h"

void task_monitor_function(void *arg) {
    test_sensor_init();

    while(1) {
        char uptime_str[32];
        test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
        ESP_LOGI(TAG, "System uptime: %s", uptime_str);

        vTaskDelay(pdMS_TO_TICKS(5000));  // Every 5 seconds
    }
}
```

## Implementation Notes

- Uses `esp_timer_get_time()` for microsecond precision
- Boot time is captured at initialization
- Elapsed time is calculated as (current_time - boot_time)
- No external hardware required
- Minimal CPU overhead
