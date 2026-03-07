#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the PTP (IEEE 1588) Client for ESP32-P4 Ethernet
 * 
 * This function initializes an internal FreeRTOS task that listens for
 * Sync, Follow_Up, Delay_Req, and Delay_Resp packets on UDP ports 319/320
 * from the Raspberry Pi 5 Grandmaster clock.
 * 
 * @note Requires the Ethernet interface (`eth0`) to be fully initialized and
 * an IP assigned via LwIP BEFORE calling this function.
 * 
 * @return ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t ptp_client_init(void);

/**
 * @brief Get the high-precision synchronized Epoch time.
 * 
 * Thread-safe function that combines the fast execution of `esp_timer_get_time()`
 * with the dynamically calculated offset (Master - Slave) updated by the PTP task.
 * 
 * This value MUST be used as the absolute timestamp for InfluxDB Line Protocol (ILP)
 * telemetry batching.
 * 
 * @return uint64_t Unix Epoch time in microseconds (µs).
 */
uint64_t get_ptp_timestamp_us(void);

/**
 * @brief Check if the PTP client is currently synchronized with the Master.
 * 
 * @return true if the offset has been calculated recently and jitter is acceptable.
 * @return false if the Master is lost or synchronization has not occurred yet.
 */
bool ptp_client_is_synced(void);

#ifdef __cplusplus
}
#endif
