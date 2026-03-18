#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the NVS partition to store PID values.
 */
esp_err_t pid_tuner_init(void);

/**
 * @brief Load Motor PIDs from Flash NVS.
 * @param index Motor index (0=Left, 1=Right)
 * @param kp Pointer to store Kp (unmodified if not found)
 * @param ki Pointer to store Ki (unmodified if not found)
 * @param kd Pointer to store Kd (unmodified if not found)
 */
esp_err_t pid_tuner_load_motor_pid(uint8_t index, float *kp, float *ki, float *kd);

/**
 * @brief Save Motor PIDs to Flash NVS directly.
 * @param index Motor index (0=Left, 1=Right)
 */
esp_err_t pid_tuner_save_motor_pid(uint8_t index, float kp, float ki, float kd);

/**
 * @brief Register the MQTT callback for incoming JSON PID tuning.
 * Must be called once before subscribing.
 */
esp_err_t pid_tuner_register_callback(void);

/**
 * @brief Subscribe to the MQTT configuration topic.
 * Must be called when MQTT connects or reconnnects.
 */
esp_err_t pid_tuner_subscribe(void);

#ifdef __cplusplus
}
#endif
