#include "performance_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "perf_mon";

esp_err_t perf_mon_init(void)
{
    ESP_LOGD(TAG, "Performance Monitor Initialized");
    return ESP_OK;
}

esp_err_t perf_mon_get_stats_absolute(perf_data_abs_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Get Total Run Time (Cumulative ticks since boot)
    // Note: portGET_RUN_TIME_COUNTER_VALUE() returns the counter for specific core.
    // However, in SMP, FreeRTOS keeps individual idle counters.
    // We will iterate over tasks to find IDLE tasks for each core or use standard stats.
    
    // For simplicity and standard ESP-IDF behavior, we use uxTaskGetSystemState to get all details.
    // But for GLOBAL CPU stats, we really want the IDLE time vs TOTAL time.
    
    // Total Runtime usually comes from a hardware timer (esp_timer or similar) used for stats.
    uint32_t total_runtime_ul = 0;
    
    // Since we need to get IDLE counts, we must snapshot tasks.
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    TaskStatus_t *pxTaskStatusArray = malloc(task_count * sizeof(TaskStatus_t));
    
    if (pxTaskStatusArray == NULL) {
        return ESP_ERR_NO_MEM;
    }

    task_count = uxTaskGetSystemState(pxTaskStatusArray, task_count, &total_runtime_ul);

    stats->cpu0_idle_ticks = 0;
    stats->cpu1_idle_ticks = 0;
    stats->cpu0_total_runtime_ticks = (uint64_t)total_runtime_ul; // Assuming shared timer base for simplified model
    stats->cpu1_total_runtime_ticks = (uint64_t)total_runtime_ul;

    for (UBaseType_t i = 0; i < task_count; i++) {
        // IDLE tasks have name "IDLE" or "IDLE0"/"IDLE1"? 
        // In ESP-IDF SMP: "IDLE0" and "IDLE1".
        if (strcmp(pxTaskStatusArray[i].pcTaskName, "IDLE0") == 0) {
            stats->cpu0_idle_ticks = pxTaskStatusArray[i].ulRunTimeCounter;
        } else if (strcmp(pxTaskStatusArray[i].pcTaskName, "IDLE1") == 0) {
            stats->cpu1_idle_ticks = pxTaskStatusArray[i].ulRunTimeCounter;
        }
    }
    
    free(pxTaskStatusArray);

    // Memory Stats
    stats->heap_free_bytes = esp_get_free_heap_size();
    stats->min_heap_free_bytes = esp_get_minimum_free_heap_size();

    // Sleep stats - placeholder if not using PM Lock/Light sleep explicitly traceable here
    stats->total_sleep_ticks = 0; 

    return ESP_OK;
}



// Static state for relative calculations
static TaskStatus_t *pxPrevTaskStatusArray = NULL;
static UBaseType_t prev_task_count = 0;
static uint32_t prev_total_runtime = 0;

// Calculated stats storage
typedef struct {
    char name[configMAX_TASK_NAME_LEN];
    int core_id; 
    float usage_pct;
} perf_record_t;

static perf_record_t *s_records = NULL;
static size_t s_record_count = 0;
static float s_core0_idle = 0.0f;
static float s_core1_idle = 0.0f;

esp_err_t perf_mon_update(void)
{
    uint32_t total_runtime;
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    
    // Allocate new snapshot
    TaskStatus_t *pxTaskStatusArray = malloc((task_count + 10) * sizeof(TaskStatus_t));
    if (pxTaskStatusArray == NULL) return ESP_ERR_NO_MEM;

    // Get current state
    task_count = uxTaskGetSystemState(pxTaskStatusArray, task_count + 10, &total_runtime);

    // If first run, store and return
    if (prev_total_runtime == 0) {
        prev_total_runtime = total_runtime;
        pxPrevTaskStatusArray = pxTaskStatusArray;
        prev_task_count = task_count;
        return ESP_ERR_INVALID_STATE; // Not enough data yet
    }

    // Allocate records
    perf_record_t *new_records = malloc(task_count * sizeof(perf_record_t));
    if (new_records == NULL) {
        free(pxTaskStatusArray);
        return ESP_ERR_NO_MEM;
    }

    // Calculate Deltas
    uint32_t total_delta = total_runtime - prev_total_runtime;
    if (total_delta == 0) total_delta = 1; 

    float c0_idle = 0.0f;
    float c1_idle = 0.0f;
    size_t valid_records = 0;

    for (UBaseType_t i = 0; i < task_count; i++) {
        TaskStatus_t *curr = &pxTaskStatusArray[i];
        
        // Find match in prev
        uint32_t prev_time = 0;
        for (UBaseType_t j = 0; j < prev_task_count; j++) {
            if (pxPrevTaskStatusArray[j].xHandle == curr->xHandle) {
                prev_time = pxPrevTaskStatusArray[j].ulRunTimeCounter;
                break;
            }
        }
        
        uint32_t delta = curr->ulRunTimeCounter - prev_time;
        float pct = ((float)delta * 100.0f) / (float)total_delta;
        
        // Accumulate IDLE
        if (strcmp(curr->pcTaskName, "IDLE0") == 0) c0_idle = pct;
        else if (strcmp(curr->pcTaskName, "IDLE1") == 0) c1_idle = pct;

        // Store in record
        if (pct > 0.0f) {
            strncpy(new_records[valid_records].name, curr->pcTaskName, configMAX_TASK_NAME_LEN - 1);
            new_records[valid_records].name[configMAX_TASK_NAME_LEN - 1] = '\0';
            new_records[valid_records].core_id = (int)curr->xCoreID;
            new_records[valid_records].usage_pct = pct;
            valid_records++;
        }
    }

    // Update global state
    if (s_records) free(s_records);
    s_records = new_records;
    s_record_count = valid_records;
    s_core0_idle = c0_idle;
    s_core1_idle = c1_idle;

    // Cleanup old snapshot
    free(pxPrevTaskStatusArray);
    
    // Update history
    pxPrevTaskStatusArray = pxTaskStatusArray;
    prev_task_count = task_count;
    prev_total_runtime = total_runtime;

    return ESP_OK;
}

void perf_mon_print_report(void)
{
    if (s_records == NULL) return;

    printf("\n\033[1;36m%-16s %-6s %8s\033[0m\n", "Task Name", "Core", "Usage %");
    printf("\033[36m----------------------------------\033[0m\n");

    for (size_t i = 0; i < s_record_count; i++) {
        char core_str[12];
        int cid = s_records[i].core_id;
        
        if (cid == 0) strcpy(core_str, "0");
        else if (cid == 1) strcpy(core_str, "1");
        else if (cid == -1 || cid == 2147483647) strcpy(core_str, "ANY");
        else snprintf(core_str, sizeof(core_str), "%d", cid);

        printf("%-16s %-6s %6.1f%%\n", s_records[i].name, core_str, s_records[i].usage_pct);
    }
    printf("\033[36m----------------------------------\033[0m\n");
    printf("CPU0 Load: \033[1;32m%5.1f%%\033[0m  (Free: %5.1f%%)\n", 100.0f - s_core0_idle, s_core0_idle);
    printf("CPU1 Load: \033[1;32m%5.1f%%\033[0m  (Free: %5.1f%%)\n", 100.0f - s_core1_idle, s_core1_idle);
    printf("\n");
}

esp_err_t perf_mon_get_report_ilp(char *buffer, size_t max_len, int64_t timestamp_ns)
{
    if (buffer == NULL || max_len == 0) return ESP_ERR_INVALID_ARG;
    if (s_records == NULL) {
        buffer[0] = '\0';
        return ESP_OK;
    }

    size_t offset = 0;
    
#ifdef CONFIG_TELEMETRY_ROBOT_NAME
    const char *robot_name = CONFIG_TELEMETRY_ROBOT_NAME;
#else
    const char *robot_name = "unknown";
#endif

    // 1. Individual Task Stats
    for (size_t i = 0; i < s_record_count; i++) {
        char core_str[12];
        int cid = s_records[i].core_id;
        
        if (cid == 0) strcpy(core_str, "0");
        else if (cid == 1) strcpy(core_str, "1");
        else if (cid == -1 || cid == 2147483647) strcpy(core_str, "ANY");
        else snprintf(core_str, sizeof(core_str), "%d", cid);

        int written = snprintf(buffer + offset, max_len - offset, 
                               "cpu_usage,task=%s,core=%s,robot=%s usage=%.1f %lld\n",
                               s_records[i].name, core_str, robot_name, s_records[i].usage_pct, timestamp_ns);
        
        if (written < 0 || (size_t)written >= max_len - offset) break;
        offset += written;
    }

    // 2. Global Core Summaries
    if (offset < max_len - 1) {
        int written = snprintf(buffer + offset, max_len - offset, 
                               "cpu_usage,task=TOTAL_CPU0,core=0,robot=%s usage=%.1f %lld\n"
                               "cpu_usage,task=TOTAL_CPU1,core=1,robot=%s usage=%.1f %lld\n",
                               robot_name, 100.0f - s_core0_idle, timestamp_ns,
                               robot_name, 100.0f - s_core1_idle, timestamp_ns);
        if (written > 0) offset += written;
    }

    if (offset < max_len) {
        buffer[offset] = '\0';
    } else {
        buffer[max_len - 1] = '\0';
    }

    return ESP_OK;
}
