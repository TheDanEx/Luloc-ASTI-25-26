#include "perf_profiler.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mqtt_custom_client.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "PROF";
bool g_prof_enabled = true;

/* ── Slot definitions ──────────────────────────────────────────── */
typedef struct {
    const char *name;       /* ILP section tag value */
    const char *category;   /* "ctrl", "uros", "mqtt", "mon" */
    float       rate_hz;
} slot_def_t;

static const slot_def_t s_defs[PROF_COUNT] = {
    [PROF_CTRL_TOTAL]        = {"ctrl_total",       "ctrl",500},
    [PROF_ENCODER_READ]      = {"encoder_read",     "ctrl",500},
    [PROF_LINE_SENSOR_READ]  = {"line_sensor",      "ctrl",500},
    [PROF_SHM_WRITE]         = {"shm_write",        "ctrl",500},
    [PROF_PID_UPDATE]        = {"pid_update",       "ctrl",500},
    [PROF_UROS_TELEM_CB]     = {"uros_telem_cb",    "uros",20},
    [PROF_UROS_DIAG_CB]      = {"uros_diag_cb",     "uros",0},
    [PROF_UROS_SYNC]         = {"uros_sync",        "uros",0},
    [PROF_UROS_PUB_SENSORS]  = {"uros_pub_sensors", "uros",20},
    [PROF_UROS_PUB_MOTORS]   = {"uros_pub_motors",  "uros",20},
    [PROF_UROS_PUB_STATUS]   = {"uros_pub_status",  "uros",20},
    [PROF_UROS_PUB_DIAG]     = {"uros_pub_diag",    "uros",0},
    [PROF_UROS_PUB_VOLTAGE]  = {"uros_pub_voltage", "uros",0},
    [PROF_UROS_SUB_CMDVEL]   = {"uros_sub_cmdvel",  "uros",0},
    [PROF_UROS_SUB_MODE]     = {"uros_sub_mode",    "uros",0},
    [PROF_MQTT_PUB_PERF]     = {"mqtt_pub_perf",    "mqtt",0},
    [PROF_MQTT_PUB_POWER]    = {"mqtt_pub_power",   "mqtt",5},
    [PROF_INA_READ]          = {"ina_read",         "mon", 5},
    [PROF_PERF_UPDATE]       = {"perf_mon_update",  "mon", 0},
    [PROF_TELEM_COMMIT]      = {"telem_commit",     "mon", 5},
    [PROF_TELEM_HF_COLLECT]  = {"telem_hf_collect", "mon", 0},
};

typedef struct {
    int64_t  min_us, max_us, total_us;
    uint32_t samples;
} prof_slot_t;

static prof_slot_t s_slots[PROF_COUNT];
static int64_t    s_start_us[PROF_COUNT];

/* ── Init ─────────────────────────────────────────────────────── */

void prof_init(void) {
    memset(s_slots, 0, sizeof(s_slots));
    ESP_LOGI(TAG, "Init, enabled=%d", g_prof_enabled);
}

void prof_set_enabled(bool on) {
    g_prof_enabled = on;
    if (!on) memset(s_slots, 0, sizeof(s_slots));
    ESP_LOGI(TAG, "Profiling %s", on ? "ON" : "OFF");
}

bool prof_is_enabled(void) { return g_prof_enabled; }

/* ── Start / End ───────────────────────────────────────────────── */

void prof_start(int slot) {
    if ((unsigned)slot >= PROF_COUNT) return;
    s_start_us[slot] = esp_timer_get_time();
}

void prof_end(int slot) {
    if ((unsigned)slot >= PROF_COUNT) return;
    int64_t dt = esp_timer_get_time() - s_start_us[slot];
    if (dt < 0) return;
    prof_slot_t *p = &s_slots[slot];
    if (p->samples == 0 || dt < p->min_us) p->min_us = dt;
    if (dt > p->max_us) p->max_us = dt;
    p->total_us += dt;
    p->samples++;
}

/* ── Console report ────────────────────────────────────────────── */

void prof_print_report(void) {
    if (!g_prof_enabled) return;
    printf("\n========== COMMS & SENSORS PROFILER ==========\n");
    printf("%-22s %4s %8s %8s %8s %6s\n",
           "SECTION","CAT","MIN(us)","AVG(us)","MAX(us)","SAMPLES");
    printf("-------------------------------------------------------------------\n");
    for (int i = 0; i < PROF_COUNT; i++) {
        if (s_slots[i].samples == 0) continue;
        printf("%-22s %4s %8lld %8lld %8lld %6lu\n",
               s_defs[i].name, s_defs[i].category,
               (long long)s_slots[i].min_us,
               (long long)(s_slots[i].total_us / s_slots[i].samples),
               (long long)s_slots[i].max_us,
               (unsigned long)s_slots[i].samples);
    }
    printf("==================================================\n\n");
    memset(s_slots, 0, sizeof(s_slots));
}

/* ── ILP export with rate limiting ──────────────────────────────── */
/*
 * ILP format:
 *   profiler,section=<name>,cat=<cat>,robot=<robot> min=<min>,avg=<avg>,max=<max>,samples=<n>i,hz=<hz> <ts_ns>
 *
 * Rate limiting: write up to `lines_per_call` ILP lines per invocation.
 * `state` tracks position across calls. Set *state=0 to start.
 * Returns bytes written (0 when no more lines).
 */

int prof_get_ilp(char *buf, int max_len, int *state, int lines_per_call,
                 const char *robot_name, int64_t ts_ns) {
    if (!g_prof_enabled || !buf || max_len < 64 || !state || !robot_name) return 0;

    int written = 0;
    int start = *state;
    int end   = start + lines_per_call;
    if (end > PROF_COUNT) end = PROF_COUNT;

    for (int i = start; i < end; i++) {
        if (s_slots[i].samples == 0) continue;
        int n = snprintf(buf + written, max_len - written,
            "profiler,section=%s,cat=%s,robot=%s "
            "min=%lldi,avg=%lldi,max=%lldi,samples=%lui,hz=%.1f %lld\n",
            s_defs[i].name, s_defs[i].category, robot_name,
            (long long)s_slots[i].min_us,
            (long long)(s_slots[i].total_us / s_slots[i].samples),
            (long long)s_slots[i].max_us,
            (unsigned long)s_slots[i].samples,
            s_defs[i].rate_hz,
            (long long)ts_ns);
        if (n < 0 || written + n >= max_len - 1) break;
        written += n;
    }

    *state = end;
    if (end >= PROF_COUNT) {
        *state = 0;
        memset(s_slots, 0, sizeof(s_slots));
    }
    return written;
}

/* ── MQTT toggle ───────────────────────────────────────────────── */

static void prof_mqtt_cb(const char *topic, int topic_len,
                         const char *data, int data_len) {
    if (!data || data_len <= 0 || data_len > 256) return;
    cJSON *r = cJSON_ParseWithLength(data, data_len);
    if (!r) return;
    cJSON *en = cJSON_GetObjectItem(r, "enabled");
    if (cJSON_IsBool(en)) prof_set_enabled(cJSON_IsTrue(en));
    else if (cJSON_IsNumber(en)) prof_set_enabled(en->valueint != 0);
    cJSON_Delete(r);
}

void prof_register_mqtt(void) {
    mqtt_custom_client_register_topic_callback(
        "robot/config/perf_profiler", prof_mqtt_cb);
    if (mqtt_custom_client_is_connected())
        mqtt_custom_client_subscribe("robot/config/perf_profiler", 0);
}
