#pragma once

/**
 * Low-power monitoring task - CPU 1
 * Low priority task for system monitoring and logging
 * Runs infrequently to minimize power consumption
 */
void task_monitor_lowpower_cpu1_start(void);
