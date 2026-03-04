/*
 * Task Monitor Low Power CPU1
 * Low-priority task for lightweight monitoring and MQTT curvature intake.
 * SPDX-License-Identifier: MIT
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "mqtt_custom_client.h"
#include "test_sensor.h"
#include "task_monitor_lowpower_cpu1.h"

static const char *TAG = "mon_cpu1";

static volatile float g_curvatura_ff = 0.0f;
static volatile uint32_t g_curvatura_ts_ms = 0;
static bool g_curvatura_callback_registered = false;
static bool g_curvatura_subscribed = false;

static void mqtt_curvatura_callback(const char *topic, int topic_len,
                                    const char *data, int data_len)
{
    (void)topic;
    (void)topic_len;

    if (data == NULL || data_len <= 0) {
        return;
    }

    bool looks_like_text = (data_len < 32);
    for (int i = 0; looks_like_text && i < data_len; i++) {
        char c = data[i];
        bool valid_char = ((c >= '0' && c <= '9') ||
                           c == '.' || c == '-' || c == '+' ||
                           c == 'e' || c == 'E' ||
                           c == ' ' || c == '\r' || c == '\n' || c == '\t');
        if (!valid_char) {
            looks_like_text = false;
        }
    }

    if (looks_like_text) {
        char payload[32] = {0};
        int copy_len = (data_len < (int)sizeof(payload) - 1) ? data_len : ((int)sizeof(payload) - 1);
        memcpy(payload, data, copy_len);

        char *endptr = NULL;
        float curv = strtof(payload, &endptr);
        if (endptr == payload) {
            ESP_LOGW(TAG, "Payload de curvatura invalido: '%s'", payload);
            return;
        }

        g_curvatura_ff = curv;
        g_curvatura_ts_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        ESP_LOGI(TAG, "Curvatura FF=%.3f (ts=%lu, txt)", g_curvatura_ff, (unsigned long)g_curvatura_ts_ms);
        return;
    }

    if (data_len >= 8) {
        uint32_t ts_ms = 0;
        float curv = 0.0f;

        memcpy(&ts_ms, data, sizeof(ts_ms));
        memcpy(&curv, data + sizeof(ts_ms), sizeof(curv));

        g_curvatura_ts_ms = ts_ms;
        g_curvatura_ff = curv;
        ESP_LOGI(TAG, "Curvatura FF=%.3f (ts=%lu, bin)", g_curvatura_ff, (unsigned long)g_curvatura_ts_ms);
        return;
    }

    ESP_LOGW(TAG, "Payload de curvatura demasiado corto (%d bytes)", data_len);
}

static void task_monitor_lowpower_cpu1(void *arg)
{
    (void)arg;

    if (test_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize test sensor");
        vTaskDelete(NULL);
    }

    char uptime_str[64];

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (mqtt_custom_client_init() == ESP_OK) {
        if (mqtt_custom_client_register_topic_callback("robot/curvatura", mqtt_curvatura_callback) == ESP_OK) {
            g_curvatura_callback_registered = true;
        } else {
            ESP_LOGE(TAG, "No se pudo registrar callback para robot/curvatura");
        }
    } else {
        ESP_LOGW(TAG, "MQTT no disponible en monitor; seguira reintentando");
    }

    while (1) {
        if (!g_curvatura_callback_registered) {
            if (mqtt_custom_client_register_topic_callback("robot/curvatura", mqtt_curvatura_callback) == ESP_OK) {
                g_curvatura_callback_registered = true;
            }
        }

        if (g_curvatura_callback_registered && !g_curvatura_subscribed && mqtt_custom_client_is_connected()) {
            if (mqtt_custom_client_subscribe("robot/curvatura", 0) >= 0) {
                g_curvatura_subscribed = true;
            }
        }

        test_sensor_get_uptime_str(uptime_str, sizeof(uptime_str));
        ESP_LOGI(TAG, "Uptime: %s | curvatura_ff=%.3f | ts=%lu",
                 uptime_str,
                 g_curvatura_ff,
                 (unsigned long)g_curvatura_ts_ms);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void task_monitor_lowpower_cpu1_start(void)
{
    xTaskCreatePinnedToCore(task_monitor_lowpower_cpu1, "monitor_cpu1", 4096, NULL, 1, NULL, 1);
}
