#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the PTP (IEEE 1588) Client
 * 
 * @return ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t ptp_client_init(void);

/**
 * @brief Get the high-precision synchronized Epoch time.
 * 
 * @return uint64_t Unix Epoch time in microseconds (µs).
 */
uint64_t get_ptp_timestamp_us(void);

/**
 * @brief Check if the PTP client is synchronized.
 * 
 * @return true if synced, false otherwise.
 */
bool ptp_client_is_synced(void);

/**
 * @brief Force a time synchronization refresh via SNTP/PTP.
 */
void ptp_client_force_sync(void);

#ifdef __cplusplus
}
#endif
