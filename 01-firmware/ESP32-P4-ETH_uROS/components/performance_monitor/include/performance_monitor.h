#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Performance Data Structure (Absolute Values)
 * 
 * Contains raw cumulative counters.
 * The backend is responsible for calculating rates/percentages over time.
 */
typedef struct {
    uint64_t cpu0_total_runtime_ticks;
    uint64_t cpu1_total_runtime_ticks;
    uint64_t cpu0_idle_ticks;
    uint64_t cpu1_idle_ticks;
    
    // Memory Status
    size_t heap_free_bytes;
    size_t min_heap_free_bytes;
    
    // Low Power Stats
    uint64_t total_sleep_ticks;
} perf_data_abs_t;

/**
 * @brief Initialize performance monitor
 * 
 * @return ESP_OK on success
 */
esp_err_t perf_mon_init(void);

/**
 * @brief Get absolute performance statistics
 * 
 * @param stats Pointer to structure to fill
 * @return ESP_OK on success
 */
esp_err_t perf_mon_get_stats_absolute(perf_data_abs_t *stats);

/**
 * @brief Get task information in JSON format
 * 
 * Generates a JSON array containing runtime stats for all tasks.
 * Format: [{"task":"name","abs_time":12345,"core":0}, ...]
 * 
 * @param buffer Buffer to store JSON string
 * @param max_len Size of buffer
 * @return ESP_OK on success
 */
esp_err_t perf_mon_get_task_info_json(char *buffer, size_t max_len);

/**
 * @brief Update performance statistics
 * 
 * Captures current task state, compares with previous, and calculates specific usage percentages.
 * Must be called periodically (e.g. every 5s).
 */
esp_err_t perf_mon_update(void);

/**
 * @brief Get the last updated report in Influx Line Protocol (ILP) format
 * 
 * Generates one line per task and one line per core.
 * Format: cpu_usage,task=name,core=0 usage=12.5 <timestamp_ns>\n
 * 
 * @param buffer Buffer to store ILP string
 * @param max_len Size of buffer
 * @param timestamp_ns Timestamp to append to each line
 * @return ESP_OK on success
 */
esp_err_t perf_mon_get_report_ilp(char *buffer, size_t max_len, int64_t timestamp_ns);

/**
 * @brief Print the last updated report to console
 */
void perf_mon_print_report(void);

#ifdef __cplusplus
}
#endif
