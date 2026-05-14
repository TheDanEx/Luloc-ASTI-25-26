#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define LIDAR_POINTS_PER_PACKET 12

typedef struct {
    float angle_deg;    // 0.0 to 360.0
    float distance_m;   // Distance in meters
    uint8_t intensity;  // 0 to 255
} lidar_point_t;

typedef struct lidar_stl19p_context_t* lidar_stl19p_handle_t;

typedef struct {
    int uart_port;      // UART_NUM_1, UART_NUM_2
    int rx_io_num;      // GPIO 14
    int baud_rate;      // 230400
    int queue_size;     // Number of points to buffer
} lidar_stl19p_config_t;

/**
 * @brief Initialize the LiDAR component
 */
esp_err_t lidar_stl19p_init(const lidar_stl19p_config_t *config, lidar_stl19p_handle_t *out_handle);

/**
 * @brief Start the reading task
 */
esp_err_t lidar_stl19p_start(lidar_stl19p_handle_t handle);

/**
 * @brief Get the queue handle to receive lidar_point_t data
 */
QueueHandle_t lidar_stl19p_get_queue(lidar_stl19p_handle_t handle);
