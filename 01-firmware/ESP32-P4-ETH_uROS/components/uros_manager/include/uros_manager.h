/**
 * @file uros_manager.h
 * @brief micro-ROS node manager for ESP32-P4.
 * 
 * This component initializes and manages the lifecycle of the micro-ROS node.
 * It handles publishers, subscribers, and time synchronization.
 */

#ifndef UROS_MANAGER_H
#define UROS_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initializes and starts the micro-ROS management task on Core 0.
 * @return ESP_OK on success, ESP_FAIL otherwise.
 */
esp_err_t uros_manager_start(void);

#endif // UROS_MANAGER_H
