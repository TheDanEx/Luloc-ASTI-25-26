#include "task_telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "telemetry_manager.h"
#include <string.h>

static const char *TAG = "TELEMETRY_TASK";
static QueueHandle_t s_telemetry_queue = NULL;
static telemetry_handle_t s_line_telemetry = NULL;

static void telemetry_task(void *arg)
{
    ESP_LOGI(TAG, "Telemetry task started");
    line_follower_telemetry_t data;

    // Use telemetry_manager to handle ILP and MQTT batching
    s_line_telemetry = telemetry_create("robot/telemetry/line", "line_follower", 50); // 50ms interval for manager
    
    while (1) {
        if (xQueueReceive(s_telemetry_queue, &data, portMAX_DELAY) == pdTRUE) {
            // Pack into telemetry_manager which handles ILP
            // Note: Use microseconds precision as requested: esp_timer_get_time()
            
            telemetry_add_int(s_line_telemetry, "mode", (int32_t)data.mode);
            telemetry_add_float(s_line_telemetry, "pos", data.position);
            telemetry_add_float(s_line_telemetry, "pid_out", data.pid_out);
            telemetry_add_float(s_line_telemetry, "ff", data.ff_val);
            telemetry_add_float(s_line_telemetry, "tgt_l", data.target_speed_l);
            telemetry_add_float(s_line_telemetry, "tgt_r", data.target_speed_r);
            telemetry_add_float(s_line_telemetry, "kp", data.kp);
            telemetry_add_float(s_line_telemetry, "ki", data.ki);
            telemetry_add_float(s_line_telemetry, "kd", data.kd);

            // Add raw and norm arrays as individual fields or strings
            // For ILP, better to use individual fields for visualization
            char key[16];
            for (int i = 0; i < 8; i++) {
                snprintf(key, sizeof(key), "raw_%d", i);
                telemetry_add_int(s_line_telemetry, key, data.raw[i]);
                snprintf(key, sizeof(key), "norm_%d", i);
                telemetry_add_float(s_line_telemetry, key, data.norm[i]);
            }

            telemetry_commit_point(s_line_telemetry);
        }
    }
}

void task_telemetry_start(void)
{
    s_telemetry_queue = xQueueCreate(20, sizeof(line_follower_telemetry_t));
    if (s_telemetry_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return;
    }

    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}

void task_telemetry_send(const line_follower_telemetry_t *data)
{
    if (s_telemetry_queue && data) {
        // Non-blocking send (timeout 0)
        xQueueSend(s_telemetry_queue, data, 0);
    }
}
