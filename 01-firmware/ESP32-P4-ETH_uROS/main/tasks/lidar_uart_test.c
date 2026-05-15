#include "lidar_uart_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// =============================================================================
// Config
// =============================================================================

static const char *TAG = "LIDAR";

#define LIDAR_UART_PORT     UART_NUM_1
#define LIDAR_RX_PIN        GPIO_NUM_14
#define LIDAR_TX_PIN        UART_PIN_NO_CHANGE

#define LIDAR_BAUDRATE      230400

#define LIDAR_HEADER        0x54
#define LIDAR_VER_LEN       0x2C

#define LIDAR_PACKET_SIZE   47
#define LIDAR_POINTS        12

#define ANGLE_TOLERANCE_DEG 3.0f
#define PRINT_PERIOD_MS     1000

// =============================================================================
// Data structs
// =============================================================================

typedef struct {
    bool valid;
    float angle_deg;
    uint16_t distance_mm;
    uint8_t confidence;
} lidar_axis_sample_t;

typedef struct {
    lidar_axis_sample_t front_0;
    lidar_axis_sample_t right_90;
    lidar_axis_sample_t back_180;
    lidar_axis_sample_t left_270;
} lidar_axis_data_t;

static lidar_axis_data_t s_axis_data = {0};

// =============================================================================
// Helpers
// =============================================================================

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static float abs_float(float x)
{
    return (x < 0.0f) ? -x : x;
}

static bool angle_near(float angle, float target, float tolerance)
{
    if (target == 0.0f) {
        return (angle <= tolerance) || (angle >= (360.0f - tolerance));
    }

    return abs_float(angle - target) <= tolerance;
}

static bool lidar_packet_basic_valid(const uint8_t packet[LIDAR_PACKET_SIZE])
{
    if (packet[0] != LIDAR_HEADER) {
        return false;
    }

    if (packet[1] != LIDAR_VER_LEN) {
        return false;
    }

    uint16_t start_angle_raw = read_u16_le(&packet[4]);
    uint16_t end_angle_raw   = read_u16_le(&packet[42]);

    // Ángulos en grados * 100. Máximo válido: 35999.
    if (start_angle_raw >= 36000 || end_angle_raw >= 36000) {
        return false;
    }

    return true;
}

static void update_axis_sample(lidar_axis_sample_t *sample,
                               float angle_deg,
                               uint16_t distance_mm,
                               uint8_t confidence)
{
    if (sample == NULL) {
        return;
    }

    // Filtrado mínimo: ignora medidas sin distancia.
    if (distance_mm == 0) {
        return;
    }

    sample->valid = true;
    sample->angle_deg = angle_deg;
    sample->distance_mm = distance_mm;
    sample->confidence = confidence;
}

static void lidar_parse_packet_update_axes(const uint8_t packet[LIDAR_PACKET_SIZE])
{
    uint16_t start_angle_raw = read_u16_le(&packet[4]);
    uint16_t end_angle_raw   = read_u16_le(&packet[42]);

    float start_angle_deg = start_angle_raw / 100.0f;
    float end_angle_deg   = end_angle_raw / 100.0f;

    float end_angle_for_interp = end_angle_deg;

    // Caso en el que el paquete cruza de 359º a 0º.
    if (end_angle_for_interp < start_angle_deg) {
        end_angle_for_interp += 360.0f;
    }

    for (int p = 0; p < LIDAR_POINTS; p++) {
        int idx = 6 + p * 3;

        uint16_t distance_mm = read_u16_le(&packet[idx]);
        uint8_t confidence   = packet[idx + 2];

        float angle = start_angle_deg +
                      ((end_angle_for_interp - start_angle_deg) * p) / (LIDAR_POINTS - 1);

        if (angle >= 360.0f) {
            angle -= 360.0f;
        }

        if (angle_near(angle, 0.0f, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.front_0, angle, distance_mm, confidence);
        }

        if (angle_near(angle, 90.0f, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.right_90, angle, distance_mm, confidence);
        }

        if (angle_near(angle, 180.0f, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.back_180, angle, distance_mm, confidence);
        }

        if (angle_near(angle, 270.0f, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.left_270, angle, distance_mm, confidence);
        }
    }
}

static void print_axis_sample(const char *name, const lidar_axis_sample_t *sample)
{
    if (sample == NULL || !sample->valid) {
        printf("%-8s | no data\n", name);
        return;
    }

    printf("%-8s | angle=%7.2f deg | distance=%5u mm | confidence=%3u\n",
           name,
           sample->angle_deg,
           sample->distance_mm,
           sample->confidence);
}

static void lidar_print_axes_once_per_second(void)
{
    static int64_t last_print_ms = 0;

    int64_t now_ms = esp_timer_get_time() / 1000;

    if ((now_ms - last_print_ms) < PRINT_PERIOD_MS) {
        return;
    }

    last_print_ms = now_ms;

    printf("\n========== LIDAR AXES ==========\n");
    print_axis_sample("0 deg FRENTE",   &s_axis_data.front_0);
    print_axis_sample("90 deg DERECHA",  &s_axis_data.right_90);
    print_axis_sample("180 deg ATRÁS", &s_axis_data.back_180);
    print_axis_sample("270 deg IZQUIERDA", &s_axis_data.left_270);
    printf("================================\n");
}

// =============================================================================
// LiDAR task
// =============================================================================

static void lidar_task(void *arg)
{
    uint8_t header = 0;
    uint8_t ver_len = 0;
    uint8_t packet[LIDAR_PACKET_SIZE];

    while (1) {

        // 1. Buscar cabecera 0x54
        int len = uart_read_bytes(
            LIDAR_UART_PORT,
            &header,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (header != LIDAR_HEADER) {
            continue;
        }

        // 2. Comprobar segundo byte 0x2C
        len = uart_read_bytes(
            LIDAR_UART_PORT,
            &ver_len,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            continue;
        }

        if (ver_len != LIDAR_VER_LEN) {
            continue;
        }

        packet[0] = header;
        packet[1] = ver_len;

        // 3. Leer el resto del paquete
        int read_len = uart_read_bytes(
            LIDAR_UART_PORT,
            &packet[2],
            LIDAR_PACKET_SIZE - 2,
            pdMS_TO_TICKS(100)
        );

        if (read_len != (LIDAR_PACKET_SIZE - 2)) {
            ESP_LOGW(TAG, "Paquete incompleto: %d bytes", read_len);
            continue;
        }

        // 4. Validación básica
        if (!lidar_packet_basic_valid(packet)) {
            continue;
        }

        // 5. Parsear puntos y actualizar valores de 0/90/180/270
        lidar_parse_packet_update_axes(packet);

        // 6. Imprimir solo cada 1 segundo
        lidar_print_axes_once_per_second();

        // Ceder CPU para no ahogar otras tareas de menor prioridad
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
// Public init
// =============================================================================

void lidar_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
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
        LIDAR_TX_PIN,
        LIDAR_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    xTaskCreatePinnedToCore(
        lidar_task,
        "lidar_task",
        4096,
        NULL,
        2,      // prioridad baja-media para no comerse el core
        NULL,
        1       // ESP32-P4: core 0 o 1, no 2
    );

    ESP_LOGI(TAG, "LiDAR UART iniciado. RX GPIO=%d, baudrate=%d",
             LIDAR_RX_PIN,
             LIDAR_BAUDRATE);
}