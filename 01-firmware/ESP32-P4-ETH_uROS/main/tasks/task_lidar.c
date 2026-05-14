#include "lidar_stl19p.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "shared_memory.h"

static const char *TAG = "TASK_LIDAR";

void task_lidar(void *arg) {
    lidar_stl19p_config_t lidar_cfg = {
        .uart_port = UART_NUM_1,
        .rx_io_num = 14,
        .baud_rate = 230400,
        .queue_size = 500 // Buffer some points
    };

    lidar_stl19p_handle_t lidar_h;
    if (lidar_stl19p_init(&lidar_cfg, &lidar_h) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LiDAR");
        vTaskDelete(NULL);
    }

    lidar_stl19p_start(lidar_h);
    QueueHandle_t q = lidar_stl19p_get_queue(lidar_h);

    lidar_point_t p;
    shared_memory_t* shm = shared_memory_get();

    while (1) {
        if (xQueueReceive(q, &p, portMAX_DELAY)) {
            // Update Shared Memory
            int angle_idx = (int)(p.angle_deg + 0.5f) % 360;
            if (angle_idx < 0) angle_idx += 360;

            if (shm != NULL && xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                shm->lidar.distances_m[angle_idx] = p.distance_m;
                shm->lidar.intensities[angle_idx] = p.intensity;
                shm->lidar.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                xSemaphoreGive(shm->mutex);
            }
        }
    }
}

void task_lidar_start(void) {
    xTaskCreatePinnedToCore(task_lidar, "task_lidar", 4096, NULL, 4, NULL, 1);
}
