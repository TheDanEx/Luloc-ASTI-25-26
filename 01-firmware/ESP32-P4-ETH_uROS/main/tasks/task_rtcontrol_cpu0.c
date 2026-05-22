#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_rtcontrol_cpu0.h"
#include "motor.h"
#include "state_machine.h"
#include "motor_velocity_ctrl.h"
#include "pid_tuner.h"
#include "shared_memory.h"
#include "modes.h"
#include <stdlib.h>
#include <math.h>
#include "encoder_sensor.h"
#include "line_sensor.h"
#include "driver/temp_sensor.h"

static const char *TAG = "rt_cntrl";

// =============================================================================
// Hardware Constraints
// =============================================================================
#define ENCODER_LEFT_PIN_A      2
#define ENCODER_LEFT_PIN_B      3
#define ENCODER_RIGHT_PIN_A     4
#define ENCODER_RIGHT_PIN_B     5
#define ENCODER_PPR             11
#define WHEEL_DIAMETER_M        0.068f
#define GEAR_RATIO              21.3f

#ifndef CONFIG_ROBOT_CONTROL_PERIOD_MS
#define CONFIG_ROBOT_CONTROL_PERIOD_MS 2
#endif

// =============================================================================
// Line Sensor Configuration
// =============================================================================
static const adc_channel_t pines_frontales[] = {
    ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_5, ADC_CHANNEL_4,
    ADC_CHANNEL_3, ADC_CHANNEL_2, ADC_CHANNEL_1, ADC_CHANNEL_0
};

static const float distancias_m[] = {
    -0.028f, -0.020f, -0.0121f, -0.0043f, 0.0043f, 0.0121f, 0.020f, 0.028f
};

// =============================================================================
// Motor Configuration
// =============================================================================

// Default HW configuration (Updated to new GPIO Table)
static motor_driver_mcpwm_t motors = {
    .left  = { .in1 = GPIO_NUM_6,  .in2 = GPIO_NUM_15},
        .right = { .in1 = GPIO_NUM_27, .in2 = GPIO_NUM_26},

    .nsleep = GPIO_NUM_NC,
    .pwm_hz = 20000,
    .resolution_hz = 10000000,  // 10 MHz
    .deadband = 30,
    .brake_on_stop = true,
};

// =============================================================================
// Real-Time Control Task
// =============================================================================

/**
 * RT Control Loop (CPU 0).
 * Responsibilities:
 * 1. Maintain velocity control (PID) for both wheels.
 * 2. Execute mode-specific logic (Calibration, Line Following, etc).
 * 3. Update shared memory state for other tasks.
 * Frequency: adaptive (runs at full speed — no artificial delay)
 */
static void task_rtcontrol_cpu0(void *arg)
{
    ESP_LOGI(TAG, "Control loop running");
    motor_mcpwm_init(&motors);
    
    // Base configuration from Kconfig (Updated via NVS/MQTT Live Tuning)
    motor_velocity_config_t cfg_l = {
        .kp              = atof(CONFIG_VELOCITY_CTRL_DEFAULT_KP),
        .ki              = atof(CONFIG_VELOCITY_CTRL_DEFAULT_KI),
        .kd              = atof(CONFIG_VELOCITY_CTRL_DEFAULT_KD),
        .max_battery_mv  = atof(CONFIG_VELOCITY_CTRL_MAX_BATTERY_MV),
        .max_motor_speed = atof(CONFIG_VELOCITY_CTRL_MAX_MOTOR_SPEED_MS),
        .deadband_v      = atof(CONFIG_VEL_CTRL_DEADBAND_V),
        .accel_limit_ms2 = atof(CONFIG_VEL_CTRL_ACCEL_LIMIT),
        .ema_alpha       = atof(CONFIG_VEL_CTRL_EMA_ALPHA)
    };
    motor_velocity_config_t cfg_r = cfg_l;
    // Baja velocidad: el derecho empuja más
    cfg_l.deadband_v += 0.20f;
    // cfg_r.deadband_v -= 0.20f;

    // Alta velocidad: el izquierdo empuja más
    cfg_l.max_motor_speed *= 1.08;

    pid_tuner_load_motor_pid(0, &cfg_l.kp, &cfg_l.ki, &cfg_l.kd);
    pid_tuner_load_motor_pid(1, &cfg_r.kp, &cfg_r.ki, &cfg_r.kd);

    motor_velocity_ctrl_handle_t ctrl_left, ctrl_right;
    motor_velocity_ctrl_create(&cfg_l, &ctrl_left);
    motor_velocity_ctrl_create(&cfg_r, &ctrl_right);

    // Initialize Wheel Encoders strictly inside CPU0 Time-Domain
    encoder_sensor_config_t enc_l_cfg = {
        .pin_a = ENCODER_LEFT_PIN_A,
        .pin_b = ENCODER_LEFT_PIN_B,
        .ppr = ENCODER_PPR,
        .wheel_diameter_m = WHEEL_DIAMETER_M,
        .gear_ratio = GEAR_RATIO,
        .reverse_direction = false
    };
    encoder_sensor_handle_t encoder_left = encoder_sensor_init(&enc_l_cfg);

    encoder_sensor_config_t enc_r_cfg = {
        .pin_a = ENCODER_RIGHT_PIN_A,
        .pin_b = ENCODER_RIGHT_PIN_B,
        .ppr = ENCODER_PPR,
        .wheel_diameter_m = WHEEL_DIAMETER_M,
        .gear_ratio = GEAR_RATIO,
        .reverse_direction = false
    };
    encoder_sensor_handle_t encoder_right = encoder_sensor_init(&enc_r_cfg);

    // Initialize Line Sensor
    line_sensor_config_t line_cfg = {
        .num_sensors = 8,
        .adc_unit = ADC_UNIT_1,
        .adc_channels = pines_frontales,
        .sensor_positions_m = distancias_m,
        .oversample_count = 0,
        .calibration_threshold = 0,
        .detection_threshold = 0.0f
    };
    line_sensor_handle_t line_array = line_sensor_init(&line_cfg);

    // Initialize ESP32 internal temperature sensor
    temp_sensor_config_t temp_cfg = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_cfg);
    temp_cfg.dac_offset = TSENS_DAC_DEFAULT;
    temp_sensor_set_config(temp_cfg);
    temp_sensor_start();
    ESP_LOGI(TAG, "Internal temperature sensor ready");
    // Calibration started by mode_calibrate.enter(), not unconditionally

    modes_init();

    const float dt_cfg = (float)CONFIG_ROBOT_CONTROL_PERIOD_MS / 1000.0f;
    float dt = dt_cfg;
    const int64_t target_period_us = CONFIG_ROBOT_CONTROL_PERIOD_MS * 1000LL;
    robot_mode_t prev_mode = MODE_NONE;

    int64_t s_prev_cycle_us = 0;
    int64_t s_cycle_min_us = INT64_MAX;
    int64_t s_cycle_max_us = 0;
    int64_t s_cycle_sum_us = 0;
    int64_t s_cycle_sum_sq = 0;
    int     s_cycle_count = 0;
    uint32_t s_overruns = 0;

    int64_t s_busy_min_us = INT64_MAX;
    int64_t s_busy_max_us = 0;
    int64_t s_busy_sum_us = 0;
    int64_t s_busy_sum_sq = 0;
    int     s_busy_count = 0;

    while(1) {
        int64_t cycle_start_us = esp_timer_get_time();

        if (s_prev_cycle_us != 0) {
            int64_t delta_us = cycle_start_us - s_prev_cycle_us;
            if (delta_us < s_cycle_min_us) s_cycle_min_us = delta_us;
            if (delta_us > s_cycle_max_us) s_cycle_max_us = delta_us;
            if (delta_us > target_period_us + 2000) s_overruns++;
            s_cycle_sum_us += delta_us;
            s_cycle_sum_sq += delta_us * delta_us;
            s_cycle_count++;
            dt = (float)delta_us / 1000000.0f;
            if (dt > 0.1f) dt = 0.1f;
        }
        s_prev_cycle_us = cycle_start_us;
        // Calibration lifecycle: start on enter, stop on exit
        robot_mode_t current_mode = state_machine_get_context()->current_mode;
        if (current_mode != prev_mode) {
            if (current_mode == MODE_CALIBRATE_LINE) {
                line_sensor_calibration_start(line_array);
            } else if (prev_mode == MODE_CALIBRATE_LINE) {
                line_sensor_calibration_stop(line_array);
            }
            prev_mode = current_mode;
        }

        // 1. High-Frequency Synchronous Encoder Polling (Eliminates Phase Lag)
        float speed_l_ms = encoder_sensor_get_speed(encoder_left);
        float distance_l_m = encoder_sensor_get_distance(encoder_left);
        float speed_r_ms = encoder_sensor_get_speed(encoder_right);
        float distance_r_m = encoder_sensor_get_distance(encoder_right);
        
        // 2. Line Sensor Polling — only when needed (not idle/teleop)
        bool need_line = (current_mode == MODE_AUTONOMOUS_PATH ||
                          current_mode == MODE_SUMO);
        // MODE_CALIBRATE_LINE: calib_task handles ADC, RT task skips to avoid contention
        line_sensor_data_t line_data;
        if (need_line) {
            line_sensor_read(line_array, &line_data);
        } else {
            memset(&line_data, 0, sizeof(line_data));
        }

        shared_memory_t* shm = shared_memory_get();
        if (shm != NULL && xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shm->sensors.motor_speed_left = speed_l_ms;
            shm->sensors.motor_distance_left = distance_l_m;
            shm->sensors.motor_speed_right = speed_r_ms;
            shm->sensors.motor_distance_right = distance_r_m;

            float chip_temp_c;
            if (temp_sensor_read_celsius(&chip_temp_c) == ESP_OK) {
                shm->sensors.temperature = chip_temp_c;
            }
            
            // Update Line Sensor SHM (only if mode needs it)
            if (need_line) {
                shm->sensors.line_detected = line_data.line_detected;
                shm->sensors.line_position_m = line_data.line_position_m;
                for (int i = 0; i < 8; i++) {
                    shm->sensors.line_norm[i] = line_data.normalized_values[i];
                    shm->sensors.line_raw[i]  = line_data.raw_values[i];
                }
                shm->sensors.line_is_calibrated = line_sensor_is_calibrated(line_array);
            }
            // Always sync calibration bounds (updated by calib_task even when RT skips ADC)
            line_sensor_get_calibration_bounds(line_array, shm->sensors.line_min, shm->sensors.line_max);
            
            xSemaphoreGive(shm->mutex);
        }

        // 2. Update PID live tuning if changes received from MQTT
        for (int i = 0; i < 2; i++) {
            float kp, ki, kd;
            if (pid_tuner_check_and_clear_update(i, &kp, &ki, &kd)) {
                motor_velocity_ctrl_set_pid((i == 0) ? ctrl_left : ctrl_right, kp, ki, kd);
            }
        }

        // 2. Execute Mode (Router Pattern / Dispatcher)
        modes_execute(&motors, ctrl_left, ctrl_right, dt);

        // Zero out PID voltages in idle mode (prevent frozen stale values)
        if (current_mode == MODE_NONE) {
            if (shm != NULL && xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                shm->sensors.motor_pid_ff_l = 0; shm->sensors.motor_pid_p_l = 0;
                shm->sensors.motor_pid_i_l = 0; shm->sensors.motor_pid_d_l = 0;
                shm->sensors.motor_pid_ff_r = 0; shm->sensors.motor_pid_p_r = 0;
                shm->sensors.motor_pid_i_r = 0; shm->sensors.motor_pid_d_r = 0;
                xSemaphoreGive(shm->mutex);
            }
        }

        // Measure busy time BEFORE publishing (this iteration's work is done)
        int64_t body_end_us = esp_timer_get_time();
        int64_t busy_us = body_end_us - cycle_start_us;
        if (busy_us < s_busy_min_us) s_busy_min_us = busy_us;
        if (busy_us > s_busy_max_us) s_busy_max_us = busy_us;
        s_busy_sum_us += busy_us;
        s_busy_sum_sq += busy_us * busy_us;
        s_busy_count++;

        // Cycle + busy time stats: publish every ~1s (skip first bogus window)
        if (s_cycle_sum_us >= 1000000LL && s_cycle_count >= 10) {
            if (shm != NULL && xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                float cyc_mean = (float)s_cycle_sum_us / (float)s_cycle_count;
                float cyc_var  = (float)s_cycle_sum_sq / (float)s_cycle_count - cyc_mean * cyc_mean;
                if (cyc_var < 0.0f) cyc_var = 0.0f;
                shm->sensors.cycle_mean_us = cyc_mean;
                shm->sensors.cycle_min_us  = (float)s_cycle_min_us;
                shm->sensors.cycle_max_us  = (float)s_cycle_max_us;
                shm->sensors.cycle_p95_us  = cyc_mean + 2.0f * sqrtf(cyc_var);
                shm->sensors.cycle_overruns = s_overruns;

                float busy_mean = (float)s_busy_sum_us / (float)s_busy_count;
                float busy_var  = (float)s_busy_sum_sq / (float)s_busy_count - busy_mean * busy_mean;
                if (busy_var < 0.0f) busy_var = 0.0f;
                shm->sensors.busy_mean_us = busy_mean;
                shm->sensors.busy_min_us  = (float)s_busy_min_us;
                shm->sensors.busy_max_us  = (float)s_busy_max_us;
                shm->sensors.busy_p95_us  = busy_mean + 2.0f * sqrtf(busy_var);
                xSemaphoreGive(shm->mutex);
            }
            s_cycle_min_us = INT64_MAX;
            s_cycle_max_us = 0;
            s_cycle_sum_us = 0;
            s_cycle_sum_sq = 0;
            s_cycle_count = 0;
            s_overruns = 0;
            s_busy_min_us = INT64_MAX;
            s_busy_max_us = 0;
            s_busy_sum_us = 0;
            s_busy_sum_sq = 0;
            s_busy_count = 0;
        }

        // Mode-dependent pacing: relax CPU in idle, full speed in active modes
        if (current_mode == MODE_NONE) {
            vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz encoder polling in idle
        }

    }
}

// =============================================================================
// Public API
// =============================================================================

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 8192, NULL, 10, NULL, 0);
}
