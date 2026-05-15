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
    uint8_t header = 0;
    uint8_t ver_len = 0;
    uint8_t packet[47];

    while (1) {

        // 1. Buscar cabecera 0x54
        int len = uart_read_bytes(
            LIDAR_UART_PORT,
            &header,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            continue;
        }

        if (header != 0x54) {
            continue;
        }

        // 2. Leer segundo byte y comprobar que es 0x2C
        len = uart_read_bytes(
            LIDAR_UART_PORT,
            &ver_len,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            continue;
        }

        if (ver_len != 0x2C) {
            continue;
        }

        // 3. Guardar cabecera en el paquete
        packet[0] = header;
        packet[1] = ver_len;

        // 4. Leer los 45 bytes restantes
        int read_len = uart_read_bytes(
            LIDAR_UART_PORT,
            &packet[2],
            45,
            pdMS_TO_TICKS(100)
        );

        if (read_len != 45) {
            ESP_LOGW(TAG, "Paquete incompleto: %d bytes", read_len);
            continue;
        }

        // 5. Imprimir paquete completo
        ESP_LOGI(TAG, "Paquete LiDAR completo:");
        ESP_LOG_BUFFER_HEX(TAG, packet, sizeof(packet));
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