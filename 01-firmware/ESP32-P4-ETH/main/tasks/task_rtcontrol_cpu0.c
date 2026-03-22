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
#include "telemetry_manager.h"
#include "encoder_sensor.h"
#include "line_sensor.h"

static const char *TAG = "rt_cntrl";

// =============================================================================
// Hardware Constraints
// =============================================================================
#define ENCODER_LEFT_PIN_A      33
#define ENCODER_LEFT_PIN_B      46
#define ENCODER_RIGHT_PIN_A     27
#define ENCODER_RIGHT_PIN_B     32
#define ENCODER_PPR             11
#define WHEEL_DIAMETER_M        0.068f
#define GEAR_RATIO              21.3f

// =============================================================================
// Line Sensor Configuration
// =============================================================================
static const adc_channel_t pines_frontales[] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
};

static const float distancias_m[] = {
    -0.05f, -0.035f, -0.02f, -0.005f, 0.005f, 0.02f, 0.035f, 0.05f
};

// =============================================================================
// Motor Configuration
// =============================================================================

// Default HW configuration (PINS: Iz: 47/48, Dr: 20/21)
static motor_driver_mcpwm_t motors = {
    .left  = { .in1 = GPIO_NUM_22, .in2 = GPIO_NUM_23},
    .right = { .in1 = GPIO_NUM_21, .in2 = GPIO_NUM_20},

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
 * Frequency: 500 Hz (2ms)
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
    line_sensor_calibration_start(line_array); // Start auto-calibration 

    modes_init();

    const float dt = (float)CONFIG_ROBOT_CONTROL_PERIOD_MS / 1000.0f;
    const TickType_t poll_rate = pdMS_TO_TICKS(CONFIG_ROBOT_CONTROL_PERIOD_MS); 

    while(1) {
        // 1. High-Frequency Synchronous Encoder Polling (Eliminates Phase Lag)
        float speed_l_ms = encoder_sensor_get_speed(encoder_left);
        float distance_l_m = encoder_sensor_get_distance(encoder_left);
        float speed_r_ms = encoder_sensor_get_speed(encoder_right);
        float distance_r_m = encoder_sensor_get_distance(encoder_right);
        
        // 2. Line Sensor Polling
        line_sensor_data_t line_data;
        line_sensor_read(line_array, &line_data);
        
        shared_memory_t* shm = shared_memory_get();
        if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shm->sensors.motor_speed_left = speed_l_ms;
            shm->sensors.motor_distance_left = distance_l_m;
            shm->sensors.motor_speed_right = speed_r_ms;
            shm->sensors.motor_distance_right = distance_r_m;

            // Update Line Sensor SHM
            shm->sensors.line_detected = line_data.line_detected;
            shm->sensors.line_position = line_data.line_position_m;
            for (int i = 0; i < 8; i++) {
                shm->sensors.line_norm[i] = line_data.normalized_values[i];
            }
            // Get calibration bounds from component internal state
            line_sensor_get_calibration_bounds(line_array, shm->sensors.line_min, shm->sensors.line_max);
            shm->sensors.line_is_calibrated = line_sensor_is_calibrated(line_array);
            
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

        vTaskDelay(poll_rate);
    }
}

// =============================================================================
// Public API
// =============================================================================

void task_rtcontrol_cpu0_start(void)
{
    xTaskCreatePinnedToCore(task_rtcontrol_cpu0, "rtcontrol_cpu0", 4096, NULL, 10, NULL, 0);
}
