#pragma once

#include "freertos/queue.h"
#include <stdbool.h>

/**
 * High-speed communications task - CPU 1 (50 Hz)
 * Handles telemetry acquisition, JSON packing, and MQTT publishing
 * Also receives commands from CPU0 via inter-core queue
 */
void task_comms_cpu1_start(void);

/**
 * Check if MQTT communications are ready
 * Returns false if MQTT initialization failed
 */
bool task_comms_cpu1_is_ready(void);

/**
 * Get the command queue for CPU0 -> CPU1 inter-core communication
 * CPU0 can send commands (up to 256 bytes) that CPU1 will process
 * Use: xQueueSend(task_comms_cpu1_get_queue(), command_data, timeout)
 */
QueueHandle_t task_comms_cpu1_get_queue(void);
