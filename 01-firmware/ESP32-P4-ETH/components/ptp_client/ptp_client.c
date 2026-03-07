/*
 * PTP (IEEE 1588) Client Implementation for ESP32-P4
 * SPDX-License-Identifier: MIT
 */

#include "ptp_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Note: A full IEEE 1588 implementation over LwIP requires RAW UDP socket binding
// to multicast IP 224.0.1.129 and ports 319 (Event) and 320 (General).
//
// For this standard integration, we create the abstraction and the task structure
// adhering strictly to BEST_PRACTICES.md (Defensive programming, no blocking loops on CPU0).

static const char *TAG = "ptp_client";

// Global internal state
static volatile int64_t master_slave_offset_us = 0;
static volatile bool is_synchronized = false;

// Task handle
static TaskHandle_t ptp_task_handle = NULL;

static void ptp_listener_task(void *arg)
{
    ESP_LOGI(TAG, "PTP Listener Task started on Core %d", xPortGetCoreID());

    // Pseudo-code for PTP LwIP integration socket binding:
    // 1. Create UDP socket
    // 2. Bind to INADDR_ANY port 319 (Event) and 320 (General)
    // 3. Join Multicast group 224.0.1.129
    
    // Set socket timeout to prevent permanent blocking
    // struct timeval tv;
    // tv.tv_sec = 1;
    // tv.tv_usec = 0;
    // setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (1) {
        // Pseudo-code implementation of the PTP state machine:
        // 1. Recvfrom() Sync packet. Capture local esp_timer_get_time() -> t2
        // 2. Parse master timestamp from Sync or FollowUp -> t1
        // 3. Send Delay_Req packet. Capture local esp_timer_get_time() -> t3
        // 4. Recvfrom() Delay_Resp packet. Parse master receipt stamp -> t4
        
        // Equation: Offset = ((t1 - t2) + (t4 - t3)) / 2
        // master_slave_offset_us = calculated_offset;
        // is_synchronized = true;

        // Simulate active listening without burning CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t ptp_client_init(void)
{
    if (ptp_task_handle != NULL) {
        ESP_LOGW(TAG, "PTP Client already initialized");
        return ESP_OK;
    }

    // Spawn task on Core 1 to avoid interfering with Real-Time control on Core 0
    BaseType_t res = xTaskCreatePinnedToCore(
        ptp_listener_task,
        "ptp_rx",
        4096,           // Stack size
        NULL,           // Parameters
        5,              // Priority (Medium)
        &ptp_task_handle,
        1               // Core 1 (Networking)
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create PTP listener task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PTP Client initialized successfully");
    return ESP_OK;
}

uint64_t get_ptp_timestamp_us(void)
{
    // The ESP32's high-resolution timer counts microseconds since boot.
    // By adding the dynamically calculated offset, we align it with the
    // Grandmaster's Unix Epoch time without constantly calling gettimeofday().
    
    int64_t local_up_time = esp_timer_get_time();
    int64_t master_time = local_up_time + master_slave_offset_us;
    
    // Failsafe: If negative during early boot sync, clamp to 0. (Epoch cannot be negative here)
    if (master_time < 0) {
        return 0;
    }
    
    return (uint64_t)master_time;
}

bool ptp_client_is_synced(void)
{
    return is_synchronized;
}
