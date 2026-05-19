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

static const char *TAG __attribute__((unused)) = "MODE_SUMO";

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
#define MIN_CONFIDENCE       5
#define WHEEL_BASE_M         0.170f
#define POINT_MAX_AGE_MS     1500
#define CONFIG_TOPIC         "robot/config/sumo"
#define TIMING_REPORT_CYCLES 250

/* ── Timing ─────────────────────────────────────────────────────── */

typedef struct {
    const char *name;
    int64_t min_us, max_us, total_us;
    uint32_t samples;
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

static void t_start(int slot) {
    s_timing[slot].total_us -= esp_timer_get_time();
}

static void t_end(int slot) {
    int64_t t = esp_timer_get_time() + s_timing[slot].total_us;
    s_timing[slot].total_us = t;
    if (t < s_timing[slot].min_us || s_timing[slot].samples == 0) {
        s_timing[slot].min_us = t;
    }
    if (t > s_timing[slot].max_us) {
        s_timing[slot].max_us = t;
    }
    s_timing[slot].samples++;
}

/* ── LiDAR scan buffers ─────────────────────────────────────────── */

static uint16_t s_lidar_scan_mm[LIDAR_SCAN_SIZE];
static uint8_t  s_lidar_scan_conf[LIDAR_SCAN_SIZE];
static uint32_t s_lidar_scan_ts_ms[LIDAR_SCAN_SIZE];
static volatile uint32_t s_packets_ok, s_packets_bad, s_points_ok, s_bytes_rx;
static TaskHandle_t s_lidar_rx_task_handle;
static bool s_uart_initialized;
static portMUX_TYPE s_lidar_mux = portMUX_INITIALIZER_UNLOCKED;
static bool giro_180;
static volatile bool stop_task;
static uint32_t s_giro_180_start_ms;
static uint16_t s_sumo_dist[LIDAR_SCAN_SIZE];
static uint8_t  s_sumo_conf[LIDAR_SCAN_SIZE];
static uint32_t s_sumo_ts[LIDAR_SCAN_SIZE];
static uint16_t s_sumo_array_lidar[VALID_SCAN_SIZE];

/* ── MQTT config ────────────────────────────────────────────────── */

typedef struct {
    float kp_v, kp_w, max_v, max_w, base_v, base_w;
    uint32_t tiempo_giro_180_ms;
    uint8_t umbral_centro;
} sumo_logic_config_t;

static sumo_logic_config_t s_current_config = {
    .kp_v = 0, .kp_w = 0.04f, .max_v = 0.5f, .max_w = 3.0f,
    .base_v = 1.5f, .base_w = 0.8f,
    .tiempo_giro_180_ms = 2000, .umbral_centro = 15
};

/* ── Helpers ────────────────────────────────────────────────────── */

static uint16_t read_u16_le(const uint8_t *d) {
    return d[0] | (d[1] << 8);
}

static uint32_t now_ms_u32(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static int norm_angle(int a) {
    while (a >= 360) a -= 360;
    while (a < 0)    a += 360;
    return a;
}

static int angle_to_idx(float deg) {
    int a = norm_angle((int)(deg + 0.5f));
    if (a >= 181 && a <= 359) return a - 181;
    return a + 179;
}

/* ── LiDAR packet logic ─────────────────────────────────────────── */

static bool pkt_valid(const uint8_t p[LIDAR_PACKET_SIZE]) {
    if (p[0] != LIDAR_HEADER || p[1] != LIDAR_VER_LEN) return false;
    uint16_t s = read_u16_le(&p[4]);
    uint16_t e = read_u16_le(&p[42]);
    if (s >= 36000 || e >= 36000) return false;
    float d = (e / 100.0f) - (s / 100.0f);
    if (d < 0) d += 360.0f;
    return (d > 0 && d <= 40.0f);
}

static void lidar_clear_scan(void) {
    portENTER_CRITICAL(&s_lidar_mux);
    memset(s_lidar_scan_mm, 0, sizeof(s_lidar_scan_mm));
    memset(s_lidar_scan_conf, 0, sizeof(s_lidar_scan_conf));
    memset(s_lidar_scan_ts_ms, 0, sizeof(s_lidar_scan_ts_ms));
    s_packets_ok = s_packets_bad = s_points_ok = s_bytes_rx = 0;
    portEXIT_CRITICAL(&s_lidar_mux);
}

static void lidar_update_point(float ang, uint16_t dist, uint8_t conf) {
    if (dist == 0 || conf < MIN_CONFIDENCE) return;
    int i = angle_to_idx(ang);
    if (i < 0 || i >= LIDAR_SCAN_SIZE) return;
    portENTER_CRITICAL(&s_lidar_mux);
    s_lidar_scan_mm[i] = dist;
    s_lidar_scan_conf[i] = conf;
    s_lidar_scan_ts_ms[i] = now_ms_u32();
    s_points_ok++;
    portEXIT_CRITICAL(&s_lidar_mux);
}

static void lidar_parse(const uint8_t p[LIDAR_PACKET_SIZE]) {
    float s = read_u16_le(&p[4]) / 100.0f;
    float e = read_u16_le(&p[42]) / 100.0f;
    float ei = e;
    if (ei < s) ei += 360.0f;
    for (int i = 0; i < LIDAR_POINTS; i++) {
        int o = 6 + i * 3;
        float ang = s + (ei - s) * i / (float)(LIDAR_POINTS - 1);
        lidar_update_point(ang, read_u16_le(&p[o]), p[o + 2]);
    }
}

static void lidar_get_copy(uint16_t d[LIDAR_SCAN_SIZE],
                           uint8_t c[LIDAR_SCAN_SIZE],
                           uint32_t t[LIDAR_SCAN_SIZE],
                           uint32_t *pok, uint32_t *pbad,
                           uint32_t *ptok, uint32_t *rx) {
    portENTER_CRITICAL(&s_lidar_mux);
    memcpy(d, s_lidar_scan_mm, sizeof(s_lidar_scan_mm));
    memcpy(c, s_lidar_scan_conf, sizeof(s_lidar_scan_conf));
    memcpy(t, s_lidar_scan_ts_ms, sizeof(s_lidar_scan_ts_ms));
    if (pok)  *pok  = s_packets_ok;
    if (pbad) *pbad = s_packets_bad;
    if (ptok) *ptok = s_points_ok;
    if (rx)   *rx   = s_bytes_rx;
    portEXIT_CRITICAL(&s_lidar_mux);
}

/* ── LiDAR RX task ──────────────────────────────────────────────── */

static void lidar_rx_task(void *arg) {
    (void)arg;
    uint8_t p[LIDAR_PACKET_SIZE];
    int pos = 0;
    while (!stop_task) {
        uint8_t b;
        int n = uart_read_bytes(LIDAR_UART_PORT, &b, 1, pdMS_TO_TICKS(20));
        if (n <= 0) {
            vTaskDelay(1);
            continue;
        }
        portENTER_CRITICAL(&s_lidar_mux);
        s_bytes_rx += n;
        portEXIT_CRITICAL(&s_lidar_mux);

        if (pos == 0) {
            if (b == LIDAR_HEADER) { p[0] = b; pos = 1; }
            continue;
        }
        if (pos == 1) {
            if (b == LIDAR_VER_LEN) { p[1] = b; pos = 2; }
            else if (b == LIDAR_HEADER) { p[0] = b; pos = 1; }
            else pos = 0;
            continue;
        }
        p[pos++] = b;
        if (pos < LIDAR_PACKET_SIZE) continue;
        pos = 0;

        if (!pkt_valid(p)) {
            portENTER_CRITICAL(&s_lidar_mux);
            s_packets_bad++;
            portEXIT_CRITICAL(&s_lidar_mux);
            continue;
        }
        portENTER_CRITICAL(&s_lidar_mux);
        s_packets_ok++;
        portEXIT_CRITICAL(&s_lidar_mux);
        lidar_parse(p);
    }
    s_lidar_rx_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ── UART init ──────────────────────────────────────────────────── */

static esp_err_t lidar_uart_start(void) {
    if (s_uart_initialized) {
        uart_flush_input(LIDAR_UART_PORT);
        return ESP_OK;
    }
    uart_config_t c = {
        .baud_rate = LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(LIDAR_UART_PORT, 8192, 0, 0, NULL, 0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    e = uart_param_config(LIDAR_UART_PORT, &c);
    if (e != ESP_OK) return e;
    e = uart_set_pin(LIDAR_UART_PORT, LIDAR_TX_PIN, LIDAR_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (e != ESP_OK) return e;
    uart_flush_input(LIDAR_UART_PORT);
    s_uart_initialized = true;
    return ESP_OK;
}

static void lidar_tasks_start(void) {
    stop_task = false;
    if (s_lidar_rx_task_handle == NULL) {
        xTaskCreatePinnedToCore(lidar_rx_task, "lidar_rx", 4096, NULL, 2,
                                &s_lidar_rx_task_handle, 1);
    }
}

static void lidar_tasks_stop(void) {
    stop_task = true;
    uart_flush_input(LIDAR_UART_PORT);
    for (int i = 0; i < 20; i++) {
        if (s_lidar_rx_task_handle == NULL) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Sumo logic ─────────────────────────────────────────────────── */

static bool find_enemy(uint16_t a[], uint16_t *pos, uint16_t *dist) {
    uint16_t best = UINT16_MAX;
    bool f = false;
    *dist = 0;
    *pos = 0;
    for (int i = 0; i < VALID_SCAN_SIZE; i++) {
        if (a[i] == 0 || a[i] >= best) continue;
        uint16_t pi = i;
        while (i < VALID_SCAN_SIZE && a[i] < OBJECT_MAX_DIST_MM) i++;
        uint16_t pe = i;
        uint32_t s = 0;
        uint16_t n = 0;
        for (int j = pi; j < pe; j++) {
            if (a[j]) { s += a[j]; n++; }
        }
        if (n == 0) continue;
        uint16_t avg = s / n;
        if (avg < best) {
            *pos = (pe + pi) / 2;
            *dist = avg;
            best = avg;
            f = true;
        }
    }
    return f;
}

static void sumo(float *vL, float *vR) {
    t_start(T_SUMO_TOTAL);
    uint32_t pok = 0, pbad = 0, ptok = 0, rx = 0;

    t_start(T_SUMO_LIDAR_COPY);
    lidar_get_copy(s_sumo_dist, s_sumo_conf, s_sumo_ts,
                   &pok, &pbad, &ptok, &rx);
    t_end(T_SUMO_LIDAR_COPY);

    t_start(T_SUMO_ARRAY_BUILD);
    uint16_t p = 0;
    uint32_t ms = now_ms_u32();
    memset(s_sumo_array_lidar, 0, sizeof(s_sumo_array_lidar));
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        if (i >= ANGLE_INI_POS && i <= ANGLE_END_POS &&
            s_sumo_dist[i] && s_sumo_ts[i] &&
            (ms - s_sumo_ts[i]) <= POINT_MAX_AGE_MS) {
            s_sumo_array_lidar[p++] = s_sumo_dist[i];
        } else if (i >= ANGLE_INI_POS && i <= ANGLE_END_POS) {
            s_sumo_array_lidar[p++] = 0;
        }
    }
    t_end(T_SUMO_ARRAY_BUILD);

    uint16_t po = 0, d0 = 0;
    t_start(T_SUMO_SEARCH);
    bool found = find_enemy(s_sumo_array_lidar, &po, &d0);
    t_end(T_SUMO_SEARCH);

    t_start(T_SUMO_KINEMATICS);
    float v = 0, w = 0;
    if (!found) {
        w = s_current_config.base_w;
        if (w == 0.0f) w = 0.5f;
    } else {
        int c = VALID_SCAN_SIZE / 2;
        int d = c - po;
        if (d > -s_current_config.umbral_centro &&
            d < s_current_config.umbral_centro) {
            w = 0;
            v = s_current_config.max_v;
        } else {
            float bw = (d < 0) ? -s_current_config.base_w
                               : s_current_config.base_w;
            w = bw + s_current_config.kp_w * d;
            if (w >  s_current_config.max_w) w =  s_current_config.max_w;
            if (w < -s_current_config.max_w) w = -s_current_config.max_w;
        }
    }
    *vL = v - (w * WHEEL_BASE_M / 2.0f);
    *vR = v + (w * WHEEL_BASE_M / 2.0f);
    t_end(T_SUMO_KINEMATICS);
    t_end(T_SUMO_TOTAL);
}

/* ── Timing report + LiDAR stats ────────────────────────────────── */

static uint32_t s_exec_cycle;

static void timing_report(void) {
    printf("\n========== SUMO EXEC TIMING (%lu cycles) ==========\n",
           (unsigned long)s_timing[0].samples);
    printf("%-22s %8s %8s %8s %6s\n",
           "SECTION", "MIN(us)", "AVG(us)", "MAX(us)", "SAMPLES");
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
    printf("---------------------------------------------------------\n");

    /* LiDAR scan freshness */
    uint32_t ms = now_ms_u32();
    uint32_t oldest = ms, newest = 0;
    int fresh = 0, stale = 0;

    portENTER_CRITICAL(&s_lidar_mux);
    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        uint32_t t = s_lidar_scan_ts_ms[i];
        if (t == 0 || ms - t > POINT_MAX_AGE_MS) {
            stale++;
            continue;
        }
        fresh++;
        if (t < oldest) oldest = t;
        if (t > newest) newest = t;
    }
    uint32_t pok  = s_packets_ok;
    uint32_t pbad = s_packets_bad;
    uint32_t ptok = s_points_ok;
    uint32_t rx   = s_bytes_rx;
    portEXIT_CRITICAL(&s_lidar_mux);

    float scan_hz = 0;
    if (fresh > 180 && newest > oldest) {
        float dt = (float)(newest - oldest) / 1000.0f;
        if (dt > 0) scan_hz = 1.0f / dt;
    }

    printf("LiDAR: fresh=%d/%d stale=%d pkts ok=%lu bad=%lu pts=%lu rx=%luB\n",
           fresh, LIDAR_SCAN_SIZE, stale,
           (unsigned long)pok, (unsigned long)pbad,
           (unsigned long)ptok, (unsigned long)rx);
    printf("LiDAR: oldest_pt=%lums newest_pt=%lums scan_hz=%.1f\n",
           (unsigned long)(ms - oldest),
           (unsigned long)(ms - newest),
           scan_hz);
    printf("==================================================\n\n");

    memset(s_timing, 0, sizeof(s_timing));
}

/* ── MQTT config callback ───────────────────────────────────────── */

static void mqtt_config_cb(const char *t, int tl, const char *d, int dl) {
    if (d == NULL || dl <= 0 || dl > 1024) return;
    cJSON *r = cJSON_ParseWithLength(d, dl);
    if (r == NULL) return;
    cJSON *kpv = cJSON_GetObjectItem(r, "kp_v");
    cJSON *kpw = cJSON_GetObjectItem(r, "kp_w");
    cJSON *mxv = cJSON_GetObjectItem(r, "max_v");
    cJSON *mxw = cJSON_GetObjectItem(r, "max_w");
    cJSON *bsv = cJSON_GetObjectItem(r, "base_v");
    cJSON *bsw = cJSON_GetObjectItem(r, "base_w");
    cJSON *tg  = cJSON_GetObjectItem(r, "tiempo_giro_180_ms");
    cJSON *uc  = cJSON_GetObjectItem(r, "umbral_centro");
    if (kpv) s_current_config.kp_v = kpv->valuedouble;
    if (kpw) s_current_config.kp_w = kpw->valuedouble;
    if (mxv) s_current_config.max_v = mxv->valuedouble;
    if (mxw) s_current_config.max_w = mxw->valuedouble;
    if (bsv) s_current_config.base_v = bsv->valuedouble;
    if (bsw) s_current_config.base_w = bsw->valuedouble;
    if (cJSON_IsNumber(tg) && tg->valueint >= 0) {
        s_current_config.tiempo_giro_180_ms = (uint32_t)tg->valueint;
    }
    if (uc) s_current_config.umbral_centro = (uint8_t)uc->valueint;
    cJSON_Delete(r);
}

/* ── Mode callbacks ─────────────────────────────────────────────── */

static void enter(void) {
    lidar_clear_scan();
    mqtt_custom_client_register_topic_callback(CONFIG_TOPIC, mqtt_config_cb);
    if (mqtt_custom_client_is_connected()) {
        mqtt_custom_client_subscribe(CONFIG_TOPIC, 0);
    }
    if (lidar_uart_start() == ESP_OK) lidar_tasks_start();
    memset(s_timing, 0, sizeof(s_timing));
    s_exec_cycle = 0;
}

static void execute(motor_driver_mcpwm_t *motors,
                    motor_velocity_ctrl_handle_t cl,
                    motor_velocity_ctrl_handle_t cr,
                    float dt) {
    t_start(T_EXEC_TOTAL);
    (void)dt;
    float vL, vR;

    t_start(T_SHM_READ);
    shared_memory_t *shm = shared_memory_get();
    if (shm == NULL) {
        t_end(T_SHM_READ);
        t_end(T_EXEC_TOTAL);
        return;
    }
    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        t_end(T_SHM_READ);
        t_end(T_EXEC_TOTAL);
        return;
    }
    bool det = shm->sensors.line_detected;
    float curl = shm->sensors.motor_speed_left;
    float curr = -shm->sensors.motor_speed_right;
    float bat = shm->sensors.battery_voltage;
    xSemaphoreGive(shm->mutex);
    t_end(T_SHM_READ);

    if (det && !giro_180) {
        giro_180 = true;
        s_giro_180_start_ms = now_ms_u32();
    }

    if (giro_180) {
        uint32_t el = now_ms_u32() - s_giro_180_start_ms;
        if (el >= s_current_config.tiempo_giro_180_ms) {
            giro_180 = false;
            vL = 0;
            vR = 0;
        } else if (el < 400) {
            vL = -s_current_config.base_v;
            vR = -s_current_config.base_v;
        } else {
            float w = s_current_config.max_w;
            if (w == 0.0f) {
                w = (s_current_config.base_w > 0.0f)
                    ? s_current_config.base_w : 0.8f;
            }
            vL = -w * WHEEL_BASE_M / 2.0f;
            vR =  w * WHEEL_BASE_M / 2.0f;
        }
    } else {
        sumo(&vL, &vR);
    }

    t_start(T_MOTOR_CTRL);
    motor_velocity_input_t ml = {
        .target_speed = vL, .current_speed = curl, .battery_mv = bat
    };
    motor_velocity_input_t mr = {
        .target_speed = vR, .current_speed = curr, .battery_mv = bat
    };
    float pl, pr;
    motor_velocity_ctrl_update(cl, &ml, dt, &pl, NULL);
    motor_velocity_ctrl_update(cr, &mr, dt, &pr, NULL);
    motor_mcpwm_set(motors, (int16_t)(pl * 10.0f), (int16_t)(pr * 10.0f));
    t_end(T_MOTOR_CTRL);

    t_end(T_EXEC_TOTAL);

    if (++s_exec_cycle >= TIMING_REPORT_CYCLES) {
        s_exec_cycle = 0;
        timing_report();
    }
}

static void exit_mode(motor_driver_mcpwm_t *m) {
    motor_mcpwm_stop(m);
    lidar_tasks_stop();
    giro_180 = false;
}

const mode_interface_t mode_sumo = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};
