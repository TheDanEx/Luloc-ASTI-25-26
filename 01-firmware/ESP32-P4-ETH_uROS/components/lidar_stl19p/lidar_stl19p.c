#include "lidar_stl19p.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LIDAR_STL19P";

#define PACKET_HEADER 0x54
#define PACKET_VER_LEN 0x2C
#define PACKET_SIZE 47
#define UART_RX_BUFFER_SIZE 2048

typedef enum {
    STATE_WAIT_HEADER,
    STATE_WAIT_VER_LEN,
    STATE_RECEIVE_PAYLOAD
} parser_state_t;

typedef struct lidar_stl19p_context_t {
    lidar_stl19p_config_t config;
    QueueHandle_t point_queue;
    uint8_t packet_buffer[PACKET_SIZE];
    int packet_index;
    parser_state_t state;
    TaskHandle_t task_handle;
} lidar_stl19p_context_t;

/**
 * @brief Calculate checksum for LD19/STL19P (Simple Sum)
 */
static uint8_t calculate_checksum(const uint8_t *data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

static void lidar_task(void *arg) {
    lidar_stl19p_context_t *ctx = (lidar_stl19p_context_t *)arg;
    uint8_t byte;
    float last_angle = 0;

    ESP_LOGI(TAG, "LiDAR task started on UART %d", ctx->config.uart_port);

    while (1) {
        // Blocking read of 1 byte
        int len = uart_read_bytes(ctx->config.uart_port, &byte, 1, portMAX_DELAY);
        if (len <= 0) continue;

        switch (ctx->state) {
            case STATE_WAIT_HEADER:
                if (byte == PACKET_HEADER) {
                    ctx->packet_buffer[0] = byte;
                    ctx->packet_index = 1;
                    ctx->state = STATE_WAIT_VER_LEN;
                }
                break;

            case STATE_WAIT_VER_LEN:
                if (byte == PACKET_VER_LEN) {
                    ctx->packet_buffer[1] = byte;
                    ctx->packet_index = 2;
                    ctx->state = STATE_RECEIVE_PAYLOAD;
                } else {
                    ctx->state = STATE_WAIT_HEADER; // False positive
                }
                break;

            case STATE_RECEIVE_PAYLOAD:
                ctx->packet_buffer[ctx->packet_index++] = byte;
                if (ctx->packet_index >= PACKET_SIZE) {
                    // Packet complete, verify checksum
                    uint8_t received_sum = ctx->packet_buffer[PACKET_SIZE - 1];
                    uint8_t calculated_sum = calculate_checksum(ctx->packet_buffer, PACKET_SIZE - 1);

                    if (received_sum == calculated_sum) {
                        // Extract data
                        uint16_t speed = ctx->packet_buffer[2] | (ctx->packet_buffer[3] << 8);
                        uint16_t start_angle_raw = ctx->packet_buffer[4] | (ctx->packet_buffer[5] << 8);
                        uint16_t end_angle_raw = ctx->packet_buffer[PACKET_SIZE - 5] | (ctx->packet_buffer[PACKET_SIZE - 4] << 8);
                        
                        float start_angle = start_angle_raw / 100.0f;
                        float end_angle = end_angle_raw / 100.0f;
                        float diff = end_angle - start_angle;
                        if (diff < 0) diff += 360.0f;
                        float step = diff / (LIDAR_POINTS_PER_PACKET - 1);

                        // Debug "Heartbeat" at 0 degrees
                        if (start_angle < last_angle) {
                            // Find distance at ~0 degrees (index 0 usually if start_angle is near 0)
                            uint16_t dist_0 = ctx->packet_buffer[6] | (ctx->packet_buffer[7] << 8);
                            ESP_LOGI(TAG, "Sweep Reset | Speed: %d deg/s | Front Dist: %d mm", speed, dist_0);
                        }
                        last_angle = start_angle;

                        // Parse 12 points
                        for (int i = 0; i < LIDAR_POINTS_PER_PACKET; i++) {
                            int offset = 6 + (i * 3);
                            uint16_t dist = ctx->packet_buffer[offset] | (ctx->packet_buffer[offset + 1] << 8);
                            uint8_t intensity = ctx->packet_buffer[offset + 2];

                            lidar_point_t p = {
                                .angle_deg = start_angle + (step * i),
                                .distance_m = dist / 1000.0f,
                                .intensity = intensity
                            };
                            if (p.angle_deg >= 360.0f) p.angle_deg -= 360.0f;

                            // Send to queue (non-blocking)
                            if (ctx->point_queue) {
                                xQueueSend(ctx->point_queue, &p, 0);
                            }
                        }
                    } else {
                        ESP_LOGD(TAG, "Checksum failed: rec=0x%02x cal=0x%02x", received_sum, calculated_sum);
                    }
                    ctx->state = STATE_WAIT_HEADER; // Reset for next packet
                }
                break;
        }
    }
}

esp_err_t lidar_stl19p_init(const lidar_stl19p_config_t *config, lidar_stl19p_handle_t *out_handle) {
    if (config == NULL || out_handle == NULL) return ESP_ERR_INVALID_ARG;

    lidar_stl19p_context_t *ctx = calloc(1, sizeof(lidar_stl19p_context_t));
    if (ctx == NULL) return ESP_ERR_NO_MEM;

    ctx->config = *config;
    ctx->state = STATE_WAIT_HEADER;

    // UART Config
    uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_port, UART_PIN_NO_CHANGE, config->rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(config->uart_port, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0));

    // Queue creation
    if (config->queue_size > 0) {
        ctx->point_queue = xQueueCreate(config->queue_size, sizeof(lidar_point_t));
    }

    *out_handle = ctx;
    return ESP_OK;
}

esp_err_t lidar_stl19p_start(lidar_stl19p_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    lidar_stl19p_context_t *ctx = (lidar_stl19p_context_t *)handle;

    xTaskCreatePinnedToCore(lidar_task, "lidar_task", 4096, ctx, 5, &ctx->task_handle, 1);
    return ESP_OK;
}

QueueHandle_t lidar_stl19p_get_queue(lidar_stl19p_handle_t handle) {
    if (handle == NULL) return NULL;
    return ((lidar_stl19p_context_t *)handle)->point_queue;
}
