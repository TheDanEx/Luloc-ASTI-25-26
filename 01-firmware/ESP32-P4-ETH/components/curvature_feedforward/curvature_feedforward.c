#include "curvature_feedforward.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "mqtt_custom_client.h"

static const char *TAG = "curv_ff";

static volatile float s_curvatura_ff = 0.0f;
static volatile uint32_t s_curvatura_ts_ms = 0;
static volatile bool s_has_curvature = false;
static bool s_callback_registered = false;
static bool s_topic_subscribed = false;

static bool is_payload_numeric_text(const char *data, int data_len)
{
    if (data_len <= 0 || data_len >= 32) {
        return false;
    }

    for (int i = 0; i < data_len; i++) {
        const char c = data[i];
        const bool valid_char = ((c >= '0' && c <= '9') ||
                                 c == '.' || c == '-' || c == '+' ||
                                 c == 'e' || c == 'E' ||
                                 c == ' ' || c == '\r' || c == '\n' || c == '\t');
        if (!valid_char) {
            return false;
        }
    }

    return true;
}

void curvature_feedforward_mqtt_callback(const char *topic, int topic_len, const char *data, int data_len)
{
    (void)topic;
    (void)topic_len;

    if (data == NULL || data_len <= 0) {
        return;
    }

    if (is_payload_numeric_text(data, data_len)) {
        char payload[32] = {0};
        const int copy_len = (data_len < (int)sizeof(payload) - 1) ? data_len : ((int)sizeof(payload) - 1);
        memcpy(payload, data, copy_len);

        char *endptr = NULL;
        const float curv = strtof(payload, &endptr);
        if (endptr == payload) {
            ESP_LOGW(TAG, "Payload de curvatura invalido: '%s'", payload);
            return;
        }

        s_curvatura_ff = curv;
        s_curvatura_ts_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        s_has_curvature = true;
        ESP_LOGI(TAG, "Curvatura FF=%.3f (ts=%lu, txt)", s_curvatura_ff, (unsigned long)s_curvatura_ts_ms);
        return;
    }

    if (data_len >= 8) {
        uint32_t ts_ms = 0;
        float curv = 0.0f;

        memcpy(&ts_ms, data, sizeof(ts_ms));
        memcpy(&curv, data + sizeof(ts_ms), sizeof(curv));

        s_curvatura_ts_ms = ts_ms;
        s_curvatura_ff = curv;
        s_has_curvature = true;
        ESP_LOGI(TAG, "Curvatura FF=%.3f (ts=%lu, bin)", s_curvatura_ff, (unsigned long)s_curvatura_ts_ms);
        return;
    }

    ESP_LOGW(TAG, "Payload de curvatura demasiado corto (%d bytes)", data_len);
}

esp_err_t curvature_feedforward_register_callback(void)
{
    if (s_callback_registered) {
        return ESP_OK;
    }

    esp_err_t ret = mqtt_custom_client_register_topic_callback(CURVATURE_FF_TOPIC, curvature_feedforward_mqtt_callback);
    if (ret == ESP_OK) {
        s_callback_registered = true;
        ESP_LOGI(TAG, "Callback registered for %s", CURVATURE_FF_TOPIC);
    } else {
        ESP_LOGW(TAG, "Failed to register callback for %s (err=0x%x)", CURVATURE_FF_TOPIC, (unsigned)ret);
    }
    return ret;
}

esp_err_t curvature_feedforward_subscribe(void)
{
    if (!s_callback_registered) {
        esp_err_t ret = curvature_feedforward_register_callback();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    if (!mqtt_custom_client_is_connected()) {
        s_topic_subscribed = false;
        return ESP_ERR_INVALID_STATE;
    }

    if (s_topic_subscribed) {
        return ESP_OK;
    }

    if (mqtt_custom_client_subscribe(CURVATURE_FF_TOPIC, 0) < 0) {
        ESP_LOGW(TAG, "Subscribe failed for %s (MQTT connected=%d)", CURVATURE_FF_TOPIC, mqtt_custom_client_is_connected() ? 1 : 0);
        return ESP_FAIL;
    }

    s_topic_subscribed = true;
    ESP_LOGI(TAG, "Subscribed to %s", CURVATURE_FF_TOPIC);
    return ESP_OK;
}

float curvature_feedforward_get_value(void)
{
    return s_curvatura_ff;
}

uint32_t curvature_feedforward_get_timestamp_ms(void)
{
    return s_curvatura_ts_ms;
}

bool curvature_feedforward_has_value(void)
{
    return s_has_curvature;
}
