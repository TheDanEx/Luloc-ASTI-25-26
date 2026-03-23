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
#include <string.h>
#include "telemetry_manager.h"
#include "encoder_sensor.h"
#include "line_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

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
// Line Sensor Constraints
// =============================================================================
#define LINE_SENSOR_MAX_CHANNELS 8
#define LINE_SENSOR_NOISE_GATE   0.05f

#ifndef CONFIG_LINE_ARRAY_SENSOR_COUNT
#define CONFIG_LINE_ARRAY_SENSOR_COUNT 4
#endif

#ifndef CONFIG_LINE_ARRAY_SENSOR_GPIO0
#define CONFIG_LINE_ARRAY_SENSOR_GPIO0 18
#endif
#ifndef CONFIG_LINE_ARRAY_SENSOR_GPIO1
#define CONFIG_LINE_ARRAY_SENSOR_GPIO1 17
#endif
#ifndef CONFIG_LINE_ARRAY_SENSOR_GPIO2
#define CONFIG_LINE_ARRAY_SENSOR_GPIO2 16
#endif
#ifndef CONFIG_LINE_ARRAY_SENSOR_GPIO3
#define CONFIG_LINE_ARRAY_SENSOR_GPIO3 19
#endif

#ifndef CONFIG_LINE_ARRAY_SENSOR_PITCH_MM
#define CONFIG_LINE_ARRAY_SENSOR_PITCH_MM 10
#endif

#ifndef CONFIG_LINE_ARRAY_EMITTER_GPIO
#define CONFIG_LINE_ARRAY_EMITTER_GPIO 5
#endif

static void line_sensor_emitter_enable(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_LINE_ARRAY_EMITTER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&io_conf) == ESP_OK) {
        (void)gpio_set_level(CONFIG_LINE_ARRAY_EMITTER_GPIO, 1);
        ESP_LOGI(TAG, "Line emitter enabled on GPIO %d", CONFIG_LINE_ARRAY_EMITTER_GPIO);
    } else {
        ESP_LOGE(TAG, "Failed to configure line emitter GPIO %d", CONFIG_LINE_ARRAY_EMITTER_GPIO);
    }
}

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
// Internal Helpers
// =============================================================================

typedef struct {
    line_sensor_handle_t handle;
    int count;
    adc_channel_t channels[LINE_SENSOR_MAX_CHANNELS];
    float positions_m[LINE_SENSOR_MAX_CHANNELS];
    adc_unit_t unit;
    bool active;
} line_sensor_bank_t;

typedef struct {
    line_sensor_bank_t adc1;
    line_sensor_bank_t adc2;
    float last_line_position_m;
} line_sensor_runtime_t;

static bool add_sensor_to_bank(line_sensor_bank_t *bank, adc_channel_t channel, float position_m)
{
    if (bank == NULL || bank->count >= LINE_SENSOR_MAX_CHANNELS) {
        return false;
    }

    bank->channels[bank->count] = channel;
    bank->positions_m[bank->count] = position_m;
    bank->count++;
    return true;
}

static bool init_bank_handle(line_sensor_bank_t *bank)
{
    if (bank == NULL || bank->count <= 0) {
        return false;
    }

    line_sensor_config_t cfg = {
        .num_sensors = bank->count,
        .adc_unit = bank->unit,
        .adc_channels = bank->channels,
        .sensor_positions_m = bank->positions_m,
        .oversample_count = 0,
        .calibration_threshold = 0,
        .detection_threshold = 0.0f,
    };

    bank->handle = line_sensor_init(&cfg);
    bank->active = (bank->handle != NULL);

    if (bank->active) {
        ESP_LOGI(TAG, "Line sensor bank initialized: unit=%d, channels=%d", (int)bank->unit, bank->count);
    } else {
        ESP_LOGE(TAG, "Failed to initialize line sensor bank: unit=%d", (int)bank->unit);
    }

    return bank->active;
}

static line_sensor_runtime_t init_line_sensor_runtime(void)
{
    line_sensor_runtime_t runtime = {0};
    runtime.adc1.unit = ADC_UNIT_1;

    // Sensor index order follows sdkconfig GPIO slots: idx 0 = rightmost, idx N-1 = leftmost.
    const int configured_gpios[] = {
        CONFIG_LINE_ARRAY_SENSOR_GPIO0,
        CONFIG_LINE_ARRAY_SENSOR_GPIO1,
        CONFIG_LINE_ARRAY_SENSOR_GPIO2,
        CONFIG_LINE_ARRAY_SENSOR_GPIO3,
    };
    const int max_configured = (int)(sizeof(configured_gpios) / sizeof(configured_gpios[0]));
    int sensor_count = CONFIG_LINE_ARRAY_SENSOR_COUNT;
    if (sensor_count < 1) {
        sensor_count = 1;
    }
    if (sensor_count > max_configured) {
        ESP_LOGW(TAG, "LINE_ARRAY_SENSOR_COUNT=%d exceeds GPIO entries (%d). Clamping.",
                 sensor_count, max_configured);
        sensor_count = max_configured;
    }

    const float pitch_m = ((float)CONFIG_LINE_ARRAY_SENSOR_PITCH_MM) / 1000.0f;
    const float center = ((float)(sensor_count - 1)) * 0.5f;

    for (int i = 0; i < sensor_count; i++) {
        const int gpio = configured_gpios[i];
        const float sensor_position = (center - (float)i) * pitch_m;

        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        esp_err_t err = adc_oneshot_io_to_channel(gpio, &unit, &channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Skipping GPIO %d (sensor idx %d): no ADC mapping", gpio, i);
            continue;
        }

        if (unit != ADC_UNIT_1) {
            ESP_LOGE(TAG, "Skipping GPIO %d (sensor idx %d): unexpected unit=%d", gpio, i, (int)unit);
            continue;
        }

        bool added = add_sensor_to_bank(&runtime.adc1, channel, sensor_position);
        if (!added) {
            ESP_LOGE(TAG, "Unable to add sensor idx %d (GPIO %d, unit %d, ch %d)", i, gpio, (int)unit, (int)channel);
        } else {
            ESP_LOGI(TAG, "Line sensor idx %d: GPIO %d -> unit %d ch %d pos %.4f m", i, gpio, (int)unit, (int)channel, sensor_position);
        }
    }

    (void)init_bank_handle(&runtime.adc1);

    if (!runtime.adc1.active) {
        ESP_LOGE(TAG, "No valid ADC1 line sensors configured");
    }

    return runtime;
}


static bool line_bank_start_calibration(line_sensor_bank_t *bank)
{
    if (bank == NULL || !bank->active || bank->handle == NULL) {
        return true;
    }
    return line_sensor_calibration_start(bank->handle) == ESP_OK;
}

static void line_bank_stop_calibration(line_sensor_bank_t *bank)
{
    if (bank == NULL || !bank->active || bank->handle == NULL) {
        return;
    }
    (void)line_sensor_calibration_stop(bank->handle);
}

static bool line_bank_is_calibrated(const line_sensor_bank_t *bank)
{
    if (bank == NULL || !bank->active || bank->handle == NULL) {
        return true;
    }
    return line_sensor_is_calibrated(bank->handle);
}

static void line_bank_accumulate(const line_sensor_bank_t *bank,
                                 bool *out_detected,
                                 float *acc_weight,
                                 float *acc_pos_weighted)
{
    if (bank == NULL || !bank->active || bank->handle == NULL) {
        return;
    }

    line_sensor_data_t data = {0};
    if (line_sensor_read(bank->handle, &data) != ESP_OK) {
        return;
    }

    if (data.line_detected) {
        *out_detected = true;
    }

    for (int i = 0; i < bank->count; i++) {
        const float w = data.normalized_values[i];
        if (w > LINE_SENSOR_NOISE_GATE) {
            *acc_weight += w;
            *acc_pos_weighted += (w * bank->positions_m[i]);
        }
    }
}

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

    line_sensor_emitter_enable();
    line_sensor_runtime_t line_runtime = init_line_sensor_runtime();
    bool line_calibration_running = false;

    modes_init();

    const float dt = (float)CONFIG_ROBOT_CONTROL_PERIOD_MS / 1000.0f;
    const TickType_t poll_rate = pdMS_TO_TICKS(CONFIG_ROBOT_CONTROL_PERIOD_MS);

    while(1) {
        // 1. High-Frequency Synchronous Encoder Polling (Eliminates Phase Lag)
        float speed_l_ms = encoder_sensor_get_speed(encoder_left);
        float distance_l_m = encoder_sensor_get_distance(encoder_left);
        float speed_r_ms = encoder_sensor_get_speed(encoder_right);
        float distance_r_m = encoder_sensor_get_distance(encoder_right);

        bool line_detected = false;
        float line_position = line_runtime.last_line_position_m;
        bool line_calibrated = false;

        const bool has_any_line_bank = line_runtime.adc1.active;
        if (has_any_line_bank) {
            const robot_mode_t active_mode = state_machine_get_context()->current_mode;
            const bool should_calibrate_line = (active_mode == MODE_CALIBRATE_LINE);

            if (should_calibrate_line && !line_calibration_running) {
                line_calibration_running = line_bank_start_calibration(&line_runtime.adc1);
                if (line_calibration_running) {
                    ESP_LOGI(TAG, "Line calibration started (ADC1)");
                } else {
                    ESP_LOGE(TAG, "Failed to start line calibration on ADC1 bank");
                }
            } else if (!should_calibrate_line && line_calibration_running) {
                line_bank_stop_calibration(&line_runtime.adc1);
                line_calibration_running = false;
                ESP_LOGI(TAG, "Line calibration stopped");
            }

            float sum_w = 0.0f;
            float sum_pos_w = 0.0f;
            line_bank_accumulate(&line_runtime.adc1, &line_detected, &sum_w, &sum_pos_w);

            if (sum_w > 0.0f) {
                line_position = sum_pos_w / sum_w;
                line_runtime.last_line_position_m = line_position;
            }

            line_calibrated = line_bank_is_calibrated(&line_runtime.adc1);
        }

        shared_memory_t* shm = shared_memory_get();
        if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shm->sensors.motor_speed_left = speed_l_ms;
            shm->sensors.motor_distance_left = distance_l_m;
            shm->sensors.motor_speed_right = speed_r_ms;
            shm->sensors.motor_distance_right = distance_r_m;
            shm->sensors.line_detected = line_detected;
            shm->sensors.line_position = line_position;
            shm->sensors.line_calibrating = line_calibration_running;
            shm->sensors.line_calibrated = line_calibrated;
            xSemaphoreGive(shm->mutex);
        }

        // 2. Update PID live tuning if changes received from MQTT
        for (int i = 0; i < 2; i++) {
            float kp, ki, kd;
            if (pid_tuner_check_and_clear_update(i, &kp, &ki, &kd)) {
                motor_velocity_ctrl_set_pid((i == 0) ? ctrl_left : ctrl_right, kp, ki, kd);
            }
        }

        // 3. Execute Mode (Router Pattern / Dispatcher)
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
