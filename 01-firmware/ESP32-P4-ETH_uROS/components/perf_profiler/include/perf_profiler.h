#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Profiling slots ─────────────────────────────────────────────── */
enum {
    PROF_CTRL_TOTAL = 0, PROF_ENCODER_READ, PROF_LINE_SENSOR_READ,
    PROF_SHM_WRITE, PROF_PID_UPDATE,
    PROF_UROS_TELEM_CB, PROF_UROS_DIAG_CB, PROF_UROS_SYNC,
    PROF_UROS_PUB_SENSORS, PROF_UROS_PUB_MOTORS, PROF_UROS_PUB_STATUS,
    PROF_UROS_PUB_DIAG, PROF_UROS_PUB_VOLTAGE,
    PROF_UROS_SUB_CMDVEL, PROF_UROS_SUB_MODE,
    PROF_MQTT_PUB_PERF, PROF_MQTT_PUB_POWER,
    PROF_INA_READ, PROF_PERF_UPDATE, PROF_TELEM_COMMIT, PROF_TELEM_HF_COLLECT,
    PROF_COUNT
};

/* ── API ─────────────────────────────────────────────────────────── */
void prof_init(void);
void prof_set_enabled(bool on);
bool prof_is_enabled(void);

void prof_start(int slot);
void prof_end(int slot);

/** Print report to console */
void prof_print_report(void);

/** Register MQTT toggle */
void prof_register_mqtt(void);

/**
 * Generate ILP lines for the profiler slots into buf.
 * Returns number of bytes written. Call repeatedly until returns 0.
 * state: caller-maintained counter (pass 0 on first call, incremented each call).
 * lines_per_call: max ILP lines per invocation (rate limiting).
 * robot_name: tag value for robot identifier.
 * ts_ns: nanosecond timestamp for all lines.
 */
int prof_get_ilp(char *buf, int max_len, int *state, int lines_per_call,
                 const char *robot_name, int64_t ts_ns);

/* ── Convenience macros ───────────────────────────────────────────── */
extern bool g_prof_enabled;
#define PROFILE_START(s)  do { if (g_prof_enabled) prof_start(s); } while(0)
#define PROFILE_END(s)    do { if (g_prof_enabled) prof_end(s); } while(0)

#ifdef __cplusplus
}
#endif
