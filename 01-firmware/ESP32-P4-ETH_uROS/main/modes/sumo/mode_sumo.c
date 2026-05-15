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
#define LIDAR_SCAN_SIZE     360

#define PRINT_PERIOD_MS     1000

#define SUMO_DETECT_DISTANCE_MM 800

#define SUMO_ATTACK_SPEED       0.45f
#define SUMO_TURN_SPEED         0.30f
#define SUMO_SEARCH_SPEED       0.22f

// =============================================================================
// Data structs
// =============================================================================

typedef struct {
    bool obj_front;
    bool obj_right;
    bool obj_left;
    bool obj_back;
} sumo_context_t;

// =============================================================================
// Static state
// =============================================================================

/*
    Mapeo del array:

    index 0   -> 181º
    index 1   -> 182º
    ...
    index 178 -> 359º
    index 179 -> 0º
    index 180 -> 1º
    ...
    index 359 -> 180º
*/
static uint16_t s_lidar_scan[LIDAR_SCAN_SIZE] = {0};

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

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static int angle_to_centered_index(float angle_deg)
{
    int angle_i = (int)(angle_deg + 0.5f);

    while (angle_i >= 360) {
        angle_i -= 360;
    }

    while (angle_i < 0) {
        angle_i += 360;
    }

    if (angle_i >= 181 && angle_i <= 359) {
        return angle_i - 181;
    }

    return angle_i + 179;
}

static int centered_index_to_angle(int index)
{
    if (index < 0 || index >= LIDAR_SCAN_SIZE) {
        return -1;
    }

    if (index <= 178) {
        return index + 181;
    }

    return index - 179;
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

    // Los ángulos vienen en grados * 100.
    if (start_angle_raw >= 36000 || end_angle_raw >= 36000) {
        return false;
    }

    return true;
}

static bool distance_is_obstacle(uint16_t distance_mm)
{
    if (distance_mm == 0) {
        return false;
    }

    if (distance_mm > SUMO_DETECT_DISTANCE_MM) {
        return false;
    }

    return true;
}

static void lidar_update_scan_point(float angle_deg, uint16_t distance_mm)
{
    if (distance_mm == 0) {
        return;
    }

    int idx = angle_to_centered_index(angle_deg);

    if (idx < 0 || idx >= LIDAR_SCAN_SIZE) {
        return;
    }

    portENTER_CRITICAL(&s_lidar_mux);
    s_lidar_scan[idx] = distance_mm;
    portEXIT_CRITICAL(&s_lidar_mux);
}

static void lidar_parse_packet_update_scan(const uint8_t packet[LIDAR_PACKET_SIZE])
{
    uint16_t start_angle_raw = read_u16_le(&packet[4]);
    uint16_t end_angle_raw   = read_u16_le(&packet[42]);

    float start_angle_deg = start_angle_raw / 100.0f;
    float end_angle_deg   = end_angle_raw / 100.0f;

    float end_angle_for_interp = end_angle_deg;

    if (end_angle_for_interp < start_angle_deg) {
        end_angle_for_interp += 360.0f;
    }

    for (int p = 0; p < LIDAR_POINTS; p++) {
        int packet_idx = 6 + p * 3;

        uint16_t distance_mm = read_u16_le(&packet[packet_idx]);

        float angle = start_angle_deg +
                      ((end_angle_for_interp - start_angle_deg) * p) / (LIDAR_POINTS - 1);

        if (angle >= 360.0f) {
            angle -= 360.0f;
        }

        lidar_update_scan_point(angle, distance_mm);
    }
}

static void lidar_get_scan_copy(uint16_t out_scan[LIDAR_SCAN_SIZE])
{
    if (out_scan == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_lidar_mux);
    memcpy(out_scan, s_lidar_scan, sizeof(s_lidar_scan));
    portEXIT_CRITICAL(&s_lidar_mux);
}

static uint16_t get_min_distance_in_angle_range_locked(int angle_start, int angle_end)
{
    uint16_t min_distance = 0;

    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        int angle = centered_index_to_angle(i);

        if (angle < 0) {
            continue;
        }

        bool in_range = false;

        if (angle_start <= angle_end) {
            in_range = (angle >= angle_start && angle <= angle_end);
        } else {
            // Rango que cruza por 0º. Ejemplo: 340º..20º
            in_range = (angle >= angle_start || angle <= angle_end);
        }

        if (!in_range) {
            continue;
        }

        uint16_t distance_mm = s_lidar_scan[i];

        if (!distance_is_obstacle(distance_mm)) {
            continue;
        }

        if (min_distance == 0 || distance_mm < min_distance) {
            min_distance = distance_mm;
        }
    }

    return min_distance;
}

static sumo_context_t logica_sumo(void)
{
    sumo_context_t context = {0};

    uint16_t front_min = 0;
    uint16_t right_min = 0;
    uint16_t left_min = 0;
    uint16_t back_min = 0;

    portENTER_CRITICAL(&s_lidar_mux);

    front_min = get_min_distance_in_angle_range_locked(340, 20);
    right_min = get_min_distance_in_angle_range_locked(40, 80);
    left_min  = get_min_distance_in_angle_range_locked(280, 320);
    back_min  = get_min_distance_in_angle_range_locked(160, 200);

    portEXIT_CRITICAL(&s_lidar_mux);

    context.obj_front = (front_min > 0);
    context.obj_right = (right_min > 0);
    context.obj_left  = (left_min > 0);
    context.obj_back  = (back_min > 0);

    return context;
}

static void lidar_print_scan_once_per_second(void)
{
    static int64_t last_print_ms = 0;

    int64_t current_ms = now_ms();

    if ((current_ms - last_print_ms) < PRINT_PERIOD_MS) {
        return;
    }

    last_print_ms = current_ms;

    uint16_t scan[LIDAR_SCAN_SIZE];
    lidar_get_scan_copy(scan);

    printf("\n========== LIDAR 360 ARRAY ==========\n");

    for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        int angle = centered_index_to_angle(i);

        printf("idx=%03d angle=%03d deg distance=%5u mm\n",
               i,
               angle,
               scan[i]);
    }

    printf("=====================================\n");
}

static void lidar_print_summary_once_per_second(void)
{
    static int64_t last_print_ms = 0;

    int64_t current_ms = now_ms();

    if ((current_ms - last_print_ms) < PRINT_PERIOD_MS) {
        return;
    }

    last_print_ms = current_ms;

    uint16_t front_min = 0;
    uint16_t right_min = 0;
    uint16_t left_min = 0;
    uint16_t back_min = 0;

    portENTER_CRITICAL(&s_lidar_mux);

    front_min = get_min_distance_in_angle_range_locked(340, 20);
    right_min = get_min_distance_in_angle_range_locked(40, 80);
    left_min  = get_min_distance_in_angle_range_locked(280, 320);
    back_min  = get_min_distance_in_angle_range_locked(160, 200);

    portEXIT_CRITICAL(&s_lidar_mux);

    printf("\n========== SUMO LIDAR SUMMARY ==========\n");
    printf("FRONT 340..020 deg: %u mm\n", front_min);
    printf("RIGHT 040..080 deg: %u mm\n", right_min);
    printf("LEFT  280..320 deg: %u mm\n", left_min);
    printf("BACK  160..200 deg: %u mm\n", back_min);
    printf("========================================\n");
}

// =============================================================================
// LiDAR task
// =============================================================================

static void lidar_task(void *arg)
{
    uint8_t packet[LIDAR_PACKET_SIZE];
    int packet_pos = 0;

    while (1) {
        uint8_t byte = 0;

        int len = uart_read_bytes(
            LIDAR_UART_PORT,
            &byte,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (packet_pos == 0) {
            if (byte == LIDAR_HEADER) {
                packet[packet_pos++] = byte;
            }
            continue;
        }

        if (packet_pos == 1) {
            if (byte == LIDAR_VER_LEN) {
                packet[packet_pos++] = byte;
            } else {
                packet_pos = 0;
            }
            continue;
        }

        packet[packet_pos++] = byte;

        if (packet_pos < LIDAR_PACKET_SIZE) {
            continue;
        }

        packet_pos = 0;

        if (!lidar_packet_basic_valid(packet)) {
            continue;
        }

        lidar_parse_packet_update_scan(packet);

        // Para depurar el array entero:
        // lidar_print_scan_once_per_second();

        // Para no saturar logs:
        lidar_print_summary_once_per_second();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
// UART init/deinit
// =============================================================================

static esp_err_t lidar_uart_start(void)
{
    if (s_uart_initialized) {
        uart_flush_input(LIDAR_UART_PORT);
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

    uart_flush_input(LIDAR_UART_PORT);

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

    portENTER_CRITICAL(&s_lidar_mux);
    memset(s_lidar_scan, 0, sizeof(s_lidar_scan));
    portEXIT_CRITICAL(&s_lidar_mux);

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
        target_left = SUMO_ATTACK_SPEED;
        target_right = SUMO_ATTACK_SPEED;
    } else if (context.obj_right) {
        target_left = SUMO_TURN_SPEED;
        target_right = -SUMO_TURN_SPEED;
    } else if (context.obj_left) {
        target_left = -SUMO_TURN_SPEED;
        target_right = SUMO_TURN_SPEED;
    } 
    // else if (context.obj_back) {
    //     target_left = SUMO_TURN_SPEED;
    //     target_right = -SUMO_TURN_SPEED;
    // } 
    else {
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