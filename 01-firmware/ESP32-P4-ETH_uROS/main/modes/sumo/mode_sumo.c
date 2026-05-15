#include "mode_interface.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "shared_memory.h"
#include "telemetry_manager.h"
#include "state_machine.h"
#include "mqtt_custom_client.h"
#include "cJSON.h"

#include "motor.h"
#include "motor_velocity_ctrl.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// =============================================================================
// Config
// =============================================================================

static const char *TAG = "SUMO_MODE";

#define LIDAR_UART_PORT     UART_NUM_1
#define LIDAR_RX_PIN        GPIO_NUM_14
#define LIDAR_TX_PIN        UART_PIN_NO_CHANGE

#define LIDAR_BAUDRATE      230400

#define LIDAR_HEADER        0x54
#define LIDAR_VER_LEN       0x2C

#define LIDAR_PACKET_SIZE   47
#define LIDAR_POINTS        12

// Ángulos reales que vas a usar para sumo.
// Ajusta estos valores tras probar con la mano/objeto.
#define SUMO_FRONT_ANGLE_DEG    0.0f
#define SUMO_RIGHT_ANGLE_DEG    50.0f
#define SUMO_LEFT_ANGLE_DEG     300.0f
#define SUMO_BACK_ANGLE_DEG     180.0f

#define ANGLE_TOLERANCE_DEG     4.0f

#define SUMO_DETECT_DISTANCE_MM 800
#define SUMO_MIN_CONFIDENCE     5
#define SUMO_SAMPLE_TIMEOUT_MS  300

#define PRINT_PERIOD_MS         1000

#define SUMO_ATTACK_SPEED       0.45f
#define SUMO_TURN_SPEED         0.30f
#define SUMO_SEARCH_SPEED       0.22f

// =============================================================================
// Data structs
// =============================================================================

typedef struct {
    bool valid;
    float angle_deg;
    uint16_t distance_mm;
    uint8_t confidence;
    int64_t timestamp_ms;
} lidar_axis_sample_t;

typedef struct {
    lidar_axis_sample_t front;
    lidar_axis_sample_t right;
    lidar_axis_sample_t back;
    lidar_axis_sample_t left;
} lidar_axis_data_t;

typedef struct {
    bool obj_front;
    bool obj_right;
    bool obj_back;
    bool obj_left;
} sumo_context_t;

// =============================================================================
// Static state
// =============================================================================

static lidar_axis_data_t s_axis_data = {0};

static TaskHandle_t s_lidar_task_handle = NULL;
static bool s_uart_initialized = false;

static portMUX_TYPE s_lidar_mux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// Helpers
// =============================================================================

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static float abs_float(float x)
{
    return (x < 0.0f) ? -x : x;
}

static bool angle_near(float angle, float target, float tolerance)
{
    if (target == 0.0f) {
        return (angle <= tolerance) || (angle >= (360.0f - tolerance));
    }

    return abs_float(angle - target) <= tolerance;
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool lidar_sample_is_fresh_and_valid(const lidar_axis_sample_t *sample)
{
    if (sample == NULL || !sample->valid) {
        return false;
    }

    if ((now_ms() - sample->timestamp_ms) > SUMO_SAMPLE_TIMEOUT_MS) {
        return false;
    }

    if (sample->distance_mm == 0) {
        return false;
    }

    if (sample->distance_mm > SUMO_DETECT_DISTANCE_MM) {
        return false;
    }

    if (sample->confidence < SUMO_MIN_CONFIDENCE) {
        return false;
    }

    return true;
}

static bool lidar_packet_basic_valid(const uint8_t packet[LIDAR_PACKET_SIZE])
{
    if (packet[0] != LIDAR_HEADER) {
        return false;
    }

    if (packet[1] != LIDAR_VER_LEN) {
        return false;
    }

    uint16_t start_angle_raw = read_u16_le(&packet[4]);
    uint16_t end_angle_raw   = read_u16_le(&packet[42]);

    // Ángulos en grados * 100.
    if (start_angle_raw >= 36000 || end_angle_raw >= 36000) {
        return false;
    }

    return true;
}

static void update_axis_sample(lidar_axis_sample_t *sample,
                               float angle_deg,
                               uint16_t distance_mm,
                               uint8_t confidence)
{
    if (sample == NULL) {
        return;
    }

    if (distance_mm == 0) {
        return;
    }

    sample->valid = true;
    sample->angle_deg = angle_deg;
    sample->distance_mm = distance_mm;
    sample->confidence = confidence;
    sample->timestamp_ms = now_ms();
}

static void lidar_parse_packet_update_axes(const uint8_t packet[LIDAR_PACKET_SIZE])
{
    uint16_t start_angle_raw = read_u16_le(&packet[4]);
    uint16_t end_angle_raw   = read_u16_le(&packet[42]);

    float start_angle_deg = start_angle_raw / 100.0f;
    float end_angle_deg   = end_angle_raw / 100.0f;

    float end_angle_for_interp = end_angle_deg;

    if (end_angle_for_interp < start_angle_deg) {
        end_angle_for_interp += 360.0f;
    }

    portENTER_CRITICAL(&s_lidar_mux);

    for (int p = 0; p < LIDAR_POINTS; p++) {
        int idx = 6 + p * 3;

        uint16_t distance_mm = read_u16_le(&packet[idx]);
        uint8_t confidence   = packet[idx + 2];

        float angle = start_angle_deg +
                      ((end_angle_for_interp - start_angle_deg) * p) / (LIDAR_POINTS - 1);

        if (angle >= 360.0f) {
            angle -= 360.0f;
        }

        if (angle_near(angle, SUMO_FRONT_ANGLE_DEG, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.front, angle, distance_mm, confidence);
        }

        if (angle_near(angle, SUMO_RIGHT_ANGLE_DEG, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.right, angle, distance_mm, confidence);
        }

        if (angle_near(angle, SUMO_LEFT_ANGLE_DEG, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.left, angle, distance_mm, confidence);
        }

        if (angle_near(angle, SUMO_BACK_ANGLE_DEG, ANGLE_TOLERANCE_DEG)) {
            update_axis_sample(&s_axis_data.back, angle, distance_mm, confidence);
        }
    }

    portEXIT_CRITICAL(&s_lidar_mux);
}

static void print_axis_sample(const char *name, const lidar_axis_sample_t *sample)
{
    if (sample == NULL || !sample->valid) {
        printf("%-10s | no data\n", name);
        return;
    }

    printf("%-10s | angle=%7.2f deg | distance=%5u mm | confidence=%3u | age=%lld ms\n",
           name,
           sample->angle_deg,
           sample->distance_mm,
           sample->confidence,
           (long long)(now_ms() - sample->timestamp_ms));
}

static void lidar_print_axes_once_per_second(void)
{
    static int64_t last_print_ms = 0;

    int64_t current_ms = now_ms();

    if ((current_ms - last_print_ms) < PRINT_PERIOD_MS) {
        return;
    }

    last_print_ms = current_ms;

    lidar_axis_data_t snapshot;

    portENTER_CRITICAL(&s_lidar_mux);
    snapshot = s_axis_data;
    portEXIT_CRITICAL(&s_lidar_mux);

    printf("\n========== SUMO LIDAR ==========\n");
    print_axis_sample("FRONT", &snapshot.front);
    print_axis_sample("RIGHT", &snapshot.right);
    print_axis_sample("LEFT",  &snapshot.left);
    print_axis_sample("BACK",  &snapshot.back);
    printf("================================\n");
}

static sumo_context_t logica_sumo(void)
{
    sumo_context_t context = {0};
    lidar_axis_data_t snapshot;

    portENTER_CRITICAL(&s_lidar_mux);
    snapshot = s_axis_data;
    portEXIT_CRITICAL(&s_lidar_mux);

    context.obj_front = lidar_sample_is_fresh_and_valid(&snapshot.front);
    context.obj_right = lidar_sample_is_fresh_and_valid(&snapshot.right);
    context.obj_left  = lidar_sample_is_fresh_and_valid(&snapshot.left);
    context.obj_back  = lidar_sample_is_fresh_and_valid(&snapshot.back);

    return context;
}

// =============================================================================
// LiDAR task
// =============================================================================

static void lidar_task(void *arg)
{
    uint8_t header = 0;
    uint8_t ver_len = 0;
    uint8_t packet[LIDAR_PACKET_SIZE];

    while (1) {
        int len = uart_read_bytes(
            LIDAR_UART_PORT,
            &header,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (header != LIDAR_HEADER) {
            continue;
        }

        len = uart_read_bytes(
            LIDAR_UART_PORT,
            &ver_len,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            continue;
        }

        if (ver_len != LIDAR_VER_LEN) {
            continue;
        }

        packet[0] = header;
        packet[1] = ver_len;

        int read_len = uart_read_bytes(
            LIDAR_UART_PORT,
            &packet[2],
            LIDAR_PACKET_SIZE - 2,
            pdMS_TO_TICKS(100)
        );

        if (read_len != (LIDAR_PACKET_SIZE - 2)) {
            ESP_LOGW(TAG, "Paquete incompleto: %d bytes", read_len);
            continue;
        }

        if (!lidar_packet_basic_valid(packet)) {
            continue;
        }

        lidar_parse_packet_update_axes(packet);
        lidar_print_axes_once_per_second();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
// UART init/deinit
// =============================================================================

static esp_err_t lidar_uart_start(void)
{
    if (s_uart_initialized) {
        return ESP_OK;
    }

    uart_config_t uart_config = {
        .baud_rate = LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;

    err = uart_driver_install(
        LIDAR_UART_PORT,
        4096,
        0,
        0,
        NULL,
        0
    );

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(LIDAR_UART_PORT, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(
        LIDAR_UART_PORT,
        LIDAR_TX_PIN,
        LIDAR_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    s_uart_initialized = true;

    ESP_LOGI(TAG, "LiDAR UART iniciado. RX GPIO=%d baud=%d",
             LIDAR_RX_PIN,
             LIDAR_BAUDRATE);

    return ESP_OK;
}

static void lidar_task_start(void)
{
    if (s_lidar_task_handle != NULL) {
        return;
    }

    xTaskCreatePinnedToCore(
        lidar_task,
        "sumo_lidar_task",
        4096,
        NULL,
        2,
        &s_lidar_task_handle,
        1
    );
}

static void lidar_task_stop(void)
{
    if (s_lidar_task_handle != NULL) {
        TaskHandle_t task = s_lidar_task_handle;
        s_lidar_task_handle = NULL;
        vTaskDelete(task);
    }
}

// =============================================================================
// Mode callbacks
// =============================================================================

static void enter(void)
{
    ESP_LOGI(TAG, "Entering SUMO mode");

    memset(&s_axis_data, 0, sizeof(s_axis_data));

    if (lidar_uart_start() == ESP_OK) {
        lidar_task_start();
    }
}

static void execute(motor_driver_mcpwm_t* motors,
                    motor_velocity_ctrl_handle_t ctrl_left,
                    motor_velocity_ctrl_handle_t ctrl_right,
                    float dt_s)
{
    shared_memory_t* shm = shared_memory_get();
    if (shm == NULL) {
        return;
    }

    float bat_mv = 16800.0f;
    float cur_spd_left = 0.0f;
    float cur_spd_right = 0.0f;

    if (xSemaphoreTake(shm->mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        bat_mv = shm->sensors.battery_voltage;
        cur_spd_left = shm->sensors.motor_speed_left;
        cur_spd_right = shm->sensors.motor_speed_right;
        xSemaphoreGive(shm->mutex);
    }

    if (bat_mv < 5000.0f) {
        bat_mv = 16800.0f;
    }

    float target_left = 0.0f;
    float target_right = 0.0f;

    sumo_context_t context = logica_sumo();

    if (context.obj_front) {
        // Atacar hacia delante
        target_left = SUMO_ATTACK_SPEED;
        target_right = SUMO_ATTACK_SPEED;
    } else if (context.obj_right) {
        // Girar hacia la derecha
        target_left = SUMO_TURN_SPEED;
        target_right = -SUMO_TURN_SPEED;
    } else if (context.obj_left) {
        // Girar hacia la izquierda
        target_left = -SUMO_TURN_SPEED;
        target_right = SUMO_TURN_SPEED;
    } else if (context.obj_back) {
        // Si lo detecta detrás, gira para buscarlo
        target_left = SUMO_TURN_SPEED;
        target_right = -SUMO_TURN_SPEED;
    } else {
        // Búsqueda si no ve nada
        target_left = SUMO_SEARCH_SPEED;
        target_right = -SUMO_SEARCH_SPEED;
    }

    motor_velocity_input_t input_l = {
        .target_speed = target_left,
        .current_speed = cur_spd_left,
        .battery_mv = bat_mv
    };

    motor_velocity_input_t input_r = {
        .target_speed = target_right,
        .current_speed = cur_spd_right,
        .battery_mv = bat_mv
    };

    float raw_pwm_l = 0.0f;
    float raw_pwm_r = 0.0f;

    motor_velocity_ctrl_update(ctrl_left,  &input_l, dt_s, &raw_pwm_l, NULL);
    motor_velocity_ctrl_update(ctrl_right, &input_r, dt_s, &raw_pwm_r, NULL);

    motor_mcpwm_set(
        motors,
        (int16_t)(raw_pwm_l * 10.0f),
        (int16_t)(raw_pwm_r * 10.0f)
    );
}

static void exit_mode(motor_driver_mcpwm_t* motors)
{
    ESP_LOGI(TAG, "Exiting SUMO mode");

    lidar_task_stop();

    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_sumo = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};