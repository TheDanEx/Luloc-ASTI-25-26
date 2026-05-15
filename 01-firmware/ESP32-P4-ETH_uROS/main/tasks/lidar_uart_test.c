#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "LIDAR";

#define LIDAR_UART_PORT 1
#define LIDAR_RX_PIN    14

#define LIDAR_BAUDRATE  230400

static void lidar_task(void *arg)
{
    uint8_t data[256];

    while (1) {

        int len = uart_read_bytes(
            LIDAR_UART_PORT,
            data,
            sizeof(data),
            pdMS_TO_TICKS(100)
        );

        if (len > 0) {
            ESP_LOGI(TAG, "Bytes recibidos: %d", len);
            ESP_LOG_BUFFER_HEX(TAG, data, len);
        }
    }
}

void lidar_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        LIDAR_UART_PORT,
        4096,
        0,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(
        LIDAR_UART_PORT,
        &uart_config
    ));

    ESP_ERROR_CHECK(uart_set_pin(
        LIDAR_UART_PORT,
        UART_PIN_NO_CHANGE, // TX no usado
        LIDAR_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    xTaskCreatePinnedToCore(
        lidar_task,
        "lidar_task",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "UART LiDAR iniciada");
}