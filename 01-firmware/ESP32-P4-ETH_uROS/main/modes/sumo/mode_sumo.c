#include "mode_interface.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "motor.h"
#include "motor_velocity_ctrl.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "mqtt_custom_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_memory.h"
#include "audio_player.h"
#include "cJSON.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "MODE_SUMO";

#define LIDAR_UART_PORT      UART_NUM_1
#define LIDAR_RX_PIN         GPIO_NUM_14
#define LIDAR_TX_PIN         UART_PIN_NO_CHANGE
#define LIDAR_BAUDRATE       230400
#define LIDAR_HEADER         0x54
#define LIDAR_VER_LEN        0x2C
#define LIDAR_PACKET_SIZE    47
#define LIDAR_POINTS         12
#define LIDAR_SCAN_SIZE      360
#define ANGLE_INI_POS        90
#define ANGLE_END_POS        270
#define VALID_SCAN_SIZE      (ANGLE_END_POS - ANGLE_INI_POS + 1)
#define OBJECT_MAX_DIST_MM   1000
#define UMBRAL_CENTRO        10
#define PRINT_PERIOD_MS      1000
#define MIN_CONFIDENCE       5
#define WHEEL_BASE_M         0.170f
#define TOTAL_GIRO_180       20000
#define POINT_MAX_AGE_MS     1500
#define CONFIG_TOPIC         "robot/config/sumo"
#define TIMING_REPORT_CYCLES 250

typedef struct {
    const char *name;
    int64_t     min_us;
    int64_t     max_us;
    int64_t     total_us;
    uint32_t    samples;
} timing_slot_t;

enum {
    T_EXEC_TOTAL = 0,
    T_SHM_READ,
    T_SUMO_TOTAL,
    T_SUMO_LIDAR_COPY,
    T_SUMO_ARRAY_BUILD,
    T_SUMO_SEARCH,
    T_SUMO_KINEMATICS,
    T_MOTOR_CTRL,
    T_COUNT
};

static timing_slot_t s_timing[T_COUNT];

static void timing_start(int slot) {
    s_timing[slot].total_us -= esp_timer_get_time();
}
static void timing_end(int slot) {
    int64_t t = esp_timer_get_time() + s_timing[slot].total_us;
    s_timing[slot].total_us = t;
    if (t < s_timing[slot].min_us || s_timing[slot].samples == 0)
        s_timing[slot].min_us = t;
    if (t > s_timing[slot].max_us)
        s_timing[slot].max_us = t;
    s_timing[slot].samples++;
}
static void timing_report(void) {
    printf("\n========== SUMO EXEC TIMING (%lu cycles) ==========\n",
           (unsigned long)s_timing[0].samples);
    printf("%-22s %8s %8s %8s %6s\n", "SECTION","MIN(us)","AVG(us)","MAX(us)","SAMPLES");
    printf("---------------------------------------------------------\n");
    for (int i = 0; i < T_COUNT; i++) {
        if (s_timing[i].samples == 0) continue;
        printf("%-22s %8lld %8lld %8lld %6lu\n",
               s_timing[i].name,
               (long long)s_timing[i].min_us,
               (long long)(s_timing[i].total_us / s_timing[i].samples),
               (long long)s_timing[i].max_us,
               (unsigned long)s_timing[i].samples);
    }
    printf("==================================================\n\n");
    memset(s_timing, 0, sizeof(s_timing));
}

static uint16_t s_lidar_scan_mm[LIDAR_SCAN_SIZE] = {0};
static uint8_t  s_lidar_scan_conf[LIDAR_SCAN_SIZE] = {0};
static uint32_t s_lidar_scan_ts_ms[LIDAR_SCAN_SIZE] = {0};
static uint16_t s_lidar_angles_valids[LIDAR_SCAN_SIZE] = {0};
static volatile uint32_t s_packets_ok = 0;
static volatile uint32_t s_packets_bad = 0;
static volatile uint32_t s_points_ok = 0;
static volatile uint32_t s_bytes_rx = 0;
static TaskHandle_t s_lidar_rx_task_handle = NULL;
static bool s_uart_initialized = false;
static portMUX_TYPE s_lidar_mux = portMUX_INITIALIZER_UNLOCKED;
static bool giro_180 = false;
static volatile bool stop_task = false;
static uint32_t s_giro_180_start_ms = 0;
static uint16_t s_sumo_dist[LIDAR_SCAN_SIZE];
static uint8_t  s_sumo_conf[LIDAR_SCAN_SIZE];
static uint32_t s_sumo_ts[LIDAR_SCAN_SIZE];
static uint16_t s_sumo_array_lidar[VALID_SCAN_SIZE];

typedef struct {
    float kp_v, kp_w, max_v, max_w, base_v, base_w;
    uint32_t tiempo_giro_180_ms;
    uint8_t  umbral_centro;
} sumo_logic_config_t;

static sumo_logic_config_t s_current_config = {
    .kp_v = 0.0f, .kp_w = 0.04f, .max_v = 0.5f, .max_w = 3.0f,
    .base_v = 1.5f, .base_w = 0.8f, .tiempo_giro_180_ms = 2000,
    .umbral_centro = 15
};

static void mqtt_config_callback(const char *topic, int topic_len,
                                 const char *data, int data_len) {
    if (data == NULL || data_len <= 0 || data_len > 1024) return;
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) return;
    cJSON *kp_v = cJSON_GetObjectItem(root, "kp_v");
    cJSON *kp_w = cJSON_GetObjectItem(root, "kp_w");
    cJSON *max_v = cJSON_GetObjectItem(root, "max_v");
    cJSON *max_w = cJSON_GetObjectItem(root, "max_w");
    cJSON *base_v = cJSON_GetObjectItem(root, "base_v");
    cJSON *base_w = cJSON_GetObjectItem(root, "base_w");
    cJSON *tgiro = cJSON_GetObjectItem(root, "tiempo_giro_180_ms");
    cJSON *umbral = cJSON_GetObjectItem(root, "umbral_centro");
    if (kp_v)    s_current_config.kp_v    = kp_v->valuedouble;
    if (kp_w)    s_current_config.kp_w    = kp_w->valuedouble;
    if (max_v)   s_current_config.max_v   = max_v->valuedouble;
    if (max_w)   s_current_config.max_w   = max_w->valuedouble;
    if (base_v)  s_current_config.base_v  = base_v->valuedouble;
    if (base_w)  s_current_config.base_w  = base_w->valuedouble;
    if (cJSON_IsNumber(tgiro) && tgiro->valueint >= 0)
        s_current_config.tiempo_giro_180_ms = (uint32_t)tgiro->valueint;
    if (umbral)  s_current_config.umbral_centro = (uint8_t)umbral->valueint;
    cJSON_Delete(root);
}

static uint16_t read_u16_le(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}
static uint32_t now_ms_u32(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}
static int normalize_angle_int(int angle) {
    while (angle >= 360) angle -= 360;
    while (angle < 0)    angle += 360;
    return angle;
}
static int angle_to_centered_index(float angle_deg) {
    int a = (int)(angle_deg + 0.5f);
    a = normalize_angle_int(a);
    if (a >= 181 && a <= 359) return a - 181;
    return a + 179;
}
static bool lidar_packet_basic_valid(const uint8_t packet[LIDAR_PACKET_SIZE]) {
    if (packet[0] != LIDAR_HEADER || packet[1] != LIDAR_VER_LEN) return false;
    uint16_t s = read_u16_le(&packet[4]);
    uint16_t e = read_u16_le(&packet[42]);
    if (s >= 36000 || e >= 36000) return false;
    float diff = (e / 100.0f) - (s / 100.0f);
    if (diff < 0) diff += 360.0f;
    return (diff > 0.0f && diff <= 40.0f);
}
static void lidar_clear_scan(void) {
    portENTER_CRITICAL(&s_lidar_mux);
    memset(s_lidar_scan_mm, 0, sizeof(s_lidar_scan_mm));
    memset(s_lidar_scan_conf, 0, sizeof(s_lidar_scan_conf));
    memset(s_lidar_scan_ts_ms, 0, sizeof(s_lidar_scan_ts_ms));
    s_packets_ok = s_packets_bad = s_points_ok = s_bytes_rx = 0;
    portEXIT_CRITICAL(&s_lidar_mux);
}
static void lidar_update_scan_point(float angle_deg, uint16_t distance_mm,
                                     uint8_t confidence) {
    if (distance_mm == 0 || confidence < MIN_CONFIDENCE) return;
    int idx = angle_to_centered_index(angle_deg);
    if (idx < 0 || idx >= LIDAR_SCAN_SIZE) return;
    uint32_t t = now_ms_u32();
    portENTER_CRITICAL(&s_lidar_mux);
    s_lidar_scan_mm[idx] = distance_mm;
    s_lidar_scan_conf[idx] = confidence;
    s_lidar_scan_ts_ms[idx] = t;
    s_points_ok++;
    portEXIT_CRITICAL(&s_lidar_mux);
}
static void lidar_parse_packet_update_scan(const uint8_t packet[LIDAR_PACKET_SIZE]) {
    uint16_t s_raw = read_u16_le(&packet[4]);
    uint16_t e_raw = read_u16_le(&packet[42]);
    float s_deg = s_raw / 100.0f;
    float e_deg = e_raw / 100.0f;
    float e_interp = e_deg;
    if (e_interp < s_deg) e_interp += 360.0f;
    for (int p = 0; p < LIDAR_POINTS; p++) {
        int off = 6 + p * 3;
        uint16_t dist = read_u16_le(&packet[off]);
        uint8_t  conf = packet[off + 2];
        float ang = s_deg + (e_interp - s_deg) * p / (float)(LIDAR_POINTS - 1);
        if (ang >= 360.0f) ang -= 360.0f;
        lidar_update_scan_point(ang, dist, conf);
    }
}
static void lidar_get_scan_copy(uint16_t out_dist[LIDAR_SCAN_SIZE],
                                uint8_t out_conf[LIDAR_SCAN_SIZE],
                                uint32_t out_ts[LIDAR_SCAN_SIZE],
                                uint32_t *out_pkts_ok, uint32_t *out_pkts_bad,
                                uint32_t *out_pts_ok, uint32_t *out_bytes_rx) {
    portENTER_CRITICAL(&s_lidar_mux);
    memcpy(out_dist, s_lidar_scan_mm, sizeof(s_lidar_scan_mm));
    memcpy(out_conf, s_lidar_scan_conf, sizeof(s_lidar_scan_conf));
    memcpy(out_ts, s_lidar_scan_ts_ms, sizeof(s_lidar_scan_ts_ms));
    if (out_pkts_ok)  *out_pkts_ok  = s_packets_ok;
    if (out_pkts_bad) *out_pkts_bad = s_packets_bad;
    if (out_pts_ok)   *out_pts_ok   = s_points_ok;
    if (out_bytes_rx) *out_bytes_rx = s_bytes_rx;
    portEXIT_CRITICAL(&s_lidar_mux);
}

static void lidar_rx_task(void *arg) {
    (void)arg;
    uint8_t packet[LIDAR_PACKET_SIZE];
    int packet_pos = 0;
    while (!stop_task) {
        uint8_t byte = 0;
        int len = uart_read_bytes(LIDAR_UART_PORT, &byte, 1, pdMS_TO_TICKS(20));
        if (len <= 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        portENTER_CRITICAL(&s_lidar_mux);
        s_bytes_rx += (uint32_t)len;
        portEXIT_CRITICAL(&s_lidar_mux);
        if (packet_pos == 0) {
            if (byte == LIDAR_HEADER) { packet[0] = byte; packet_pos = 1; }
            continue;
        }
        if (packet_pos == 1) {
            if (byte == LIDAR_VER_LEN) { packet[1] = byte; packet_pos = 2; }
            else if (byte == LIDAR_HEADER) { packet[0] = byte; packet_pos = 1; }
            else { packet_pos = 0; }
            continue;
        }
        packet[packet_pos++] = byte;
        if (packet_pos < LIDAR_PACKET_SIZE) continue;
        packet_pos = 0;
        if (!lidar_packet_basic_valid(packet)) {
            portENTER_CRITICAL(&s_lidar_mux);
            s_packets_bad++;
            portEXIT_CRITICAL(&s_lidar_mux);
            continue;
        }
        portENTER_CRITICAL(&s_lidar_mux);
        s_packets_ok++;
        portEXIT_CRITICAL(&s_lidar_mux);
        lidar_parse_packet_update_scan(packet);
    }
    s_lidar_rx_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t lidar_uart_start(void) {
    if (s_uart_initialized) { uart_flush_input(LIDAR_UART_PORT); return ESP_OK; }
    uart_config_t cfg = {
        .baud_rate = LIDAR_BAUDRATE, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(LIDAR_UART_PORT, 8192, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = uart_param_config(LIDAR_UART_PORT, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(LIDAR_UART_PORT, LIDAR_TX_PIN, LIDAR_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    uart_flush_input(LIDAR_UART_PORT);
    s_uart_initialized = true;
    ESP_LOGI(TAG, "LiDAR UART RX=GPIO%d @ %d baud", LIDAR_RX_PIN, LIDAR_BAUDRATE);
    return ESP_OK;
}
static void lidar_tasks_start(void) {
    stop_task = false;
    if (s_lidar_rx_task_handle == NULL)
        xTaskCreatePinnedToCore(lidar_rx_task, "lidar_rx", 4096, NULL, 2,
                                 &s_lidar_rx_task_handle, 1);
}
static void lidar_tasks_stop(void) {
    stop_task = true;
    uart_flush_input(LIDAR_UART_PORT);
    for (int i = 0; i < 20; i++) {
        if (s_lidar_rx_task_handle == NULL) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool find_closest_object(uint16_t array_lidar[], uint16_t *pos_object,
                                uint16_t *dist_object) {
    uint16_t best = UINT16_MAX;
    bool found = false;
    *dist_object = 0; *pos_object = 0;
    for (int i = 0; i < VALID_SCAN_SIZE; i++) {
        if (array_lidar[i] == 0 || array_lidar[i] >= best) continue;
        uint16_t pi = i;
        while (i < VALID_SCAN_SIZE && array_lidar[i] < OBJECT_MAX_DIST_MM) i++;
        uint16_t pe = i;
        uint32_t sum = 0; uint16_t pts = 0;
        for (int j = pi; j < pe; j++)
            if (array_lidar[j] > 0) { sum += array_lidar[j]; pts++; }
        if (pts == 0) continue;
        uint16_t avg = sum / pts;
        if (avg < best) {
            *pos_object = (pe + pi) / 2;
            *dist_object = avg;
            best = avg;
            found = true;
        }
    }
    return found;
}

static void sumo(float *vL, float *vR) {
    timing_start(T_SUMO_TOTAL);
    uint32_t pkts_ok = 0, pkts_bad = 0, pts_ok = 0, bytes_rx = 0;
    timing_start(T_SUMO_LIDAR_COPY);
    lidar_get_scan_copy(s_sumo_dist, s_sumo_conf, s_sumo_ts,
                        &pkts_ok, &pkts_bad, &pts_ok, &bytes_rx);
    timing_end(T_SUMO_LIDAR_COPY);
    timing_start(T_SUMO_ARRAY_BUILD);
    uint16_t pos = 0;
    uint32_t ms_now = now_ms_u32();
    memset(s_sumo_array_lidar, 0, sizeof(s_sumo_array_lidar));
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        if (i >= ANGLE_INI_POS && i <= ANGLE_END_POS &&
            s_sumo_dist[i] > 0 && s_sumo_ts[i] > 0 &&
            (ms_now - s_sumo_ts[i]) <= POINT_MAX_AGE_MS)
            s_sumo_array_lidar[pos++] = s_sumo_dist[i];
        else if (i >= ANGLE_INI_POS && i <= ANGLE_END_POS)
            s_sumo_array_lidar[pos++] = 0;
    }
    timing_end(T_SUMO_ARRAY_BUILD);
    uint16_t pos_obj = 0, dist_obj = 0;
    timing_start(T_SUMO_SEARCH);
    bool found = find_closest_object(s_sumo_array_lidar, &pos_obj, &dist_obj);
    timing_end(T_SUMO_SEARCH);
    timing_start(T_SUMO_KINEMATICS);
    float v = 0, w = 0;
    if (!found) {
        w = s_current_config.base_w;
        if (w == 0.0f) w = 0.5f;
    } else {
        int centro = VALID_SCAN_SIZE / 2;
        int diff = centro - pos_obj;
        if (diff > -s_current_config.umbral_centro &&
            diff < s_current_config.umbral_centro) {
            w = 0; v = s_current_config.max_v;
        } else {
            float bw = (diff < 0) ? -s_current_config.base_w : s_current_config.base_w;
            w = bw + s_current_config.kp_w * diff;
            if (w >  s_current_config.max_w) w =  s_current_config.max_w;
            if (w < -s_current_config.max_w) w = -s_current_config.max_w;
        }
    }
    *vL = v - (w * WHEEL_BASE_M / 2.0f);
    *vR = v + (w * WHEEL_BASE_M / 2.0f);
    timing_end(T_SUMO_KINEMATICS);
    timing_end(T_SUMO_TOTAL);
}

static uint32_t s_exec_cycle = 0;

static void enter(void) {
    lidar_clear_scan();
    mqtt_custom_client_register_topic_callback(CONFIG_TOPIC, mqtt_config_callback);
    if (mqtt_custom_client_is_connected())
        mqtt_custom_client_subscribe(CONFIG_TOPIC, 0);
    if (lidar_uart_start() == ESP_OK) lidar_tasks_start();
    memset(s_timing, 0, sizeof(s_timing));
    s_exec_cycle = 0;
}

static void execute(motor_driver_mcpwm_t *motors,
                    motor_velocity_ctrl_handle_t ctrl_left,
                    motor_velocity_ctrl_handle_t ctrl_right,
                    float dt_s) {
    timing_start(T_EXEC_TOTAL);
    (void)dt_s;
    float vL, vR;
    timing_start(T_SHM_READ);
    shared_memory_t *shm = shared_memory_get();
    if (shm == NULL) { timing_end(T_SHM_READ); timing_end(T_EXEC_TOTAL); return; }
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        timing_end(T_SHM_READ); timing_end(T_EXEC_TOTAL); return;
    }
    bool detected = shm->sensors.line_detected;
    float cur_l = shm->sensors.motor_speed_left;
    float cur_r = -shm->sensors.motor_speed_right;
    float bat_mv = shm->sensors.battery_voltage;
    xSemaphoreGive(shm->mutex);
    timing_end(T_SHM_READ);

    if (detected && !giro_180) {
        giro_180 = true;
        s_giro_180_start_ms = now_ms_u32();
    }
    if (giro_180) {
        uint32_t elapsed = now_ms_u32() - s_giro_180_start_ms;
        if (elapsed >= s_current_config.tiempo_giro_180_ms) {
            giro_180 = false; vL = 0; vR = 0;
        } else if (elapsed < 400) {
            vL = -s_current_config.base_v;
            vR = -s_current_config.base_v;
        } else {
            float w = s_current_config.max_w;
            if (w == 0.0f) w = s_current_config.base_w > 0.0f ? s_current_config.base_w : 0.8f;
            vL = -w * WHEEL_BASE_M / 2.0f;
            vR =  w * WHEEL_BASE_M / 2.0f;
        }
    } else {
        sumo(&vL, &vR);
    }

    timing_start(T_MOTOR_CTRL);
    motor_velocity_input_t ml = { .target_speed = vL, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t mr = { .target_speed = vR, .current_speed = cur_r, .battery_mv = bat_mv };
    float pwml, pwmr;
    motor_velocity_ctrl_update(ctrl_left,  &ml, dt_s, &pwml, NULL);
    motor_velocity_ctrl_update(ctrl_right, &mr, dt_s, &pwmr, NULL);
    motor_mcpwm_set(motors, (int16_t)(pwml * 10.0f), (int16_t)(pwmr * 10.0f));
    timing_end(T_MOTOR_CTRL);
    timing_end(T_EXEC_TOTAL);

    s_exec_cycle++;
    if (s_exec_cycle >= TIMING_REPORT_CYCLES) {
        s_exec_cycle = 0;
        timing_report();
    }
}

static void exit_mode(motor_driver_mcpwm_t *motors) {
    motor_mcpwm_stop(motors);
    lidar_tasks_stop();
    giro_180 = false;
}

const mode_interface_t mode_sumo = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};
