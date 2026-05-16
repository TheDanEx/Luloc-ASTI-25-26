#include "mode_interface.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "motor.h"
#include "motor_velocity_ctrl.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "mqtt_custom_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_memory.h"
#include "telemetry_manager.h"

#include "cJSON.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
// =============================================================================
// Config
// =============================================================================

static const char *TAG = "MODE_SUMO";

#define LIDAR_UART_PORT      UART_NUM_1
#define LIDAR_RX_PIN         GPIO_NUM_14
#define LIDAR_TX_PIN         UART_PIN_NO_CHANGE

#define LIDAR_BAUDRATE       230400

#define LIDAR_HEADER         0x54
#define LIDAR_VER_LEN        0x2C

#define LIDAR_PACKET_SIZE    47
#define LIDAR_POINTS         12
#define LIDAR_SCAN_SIZE      360

#define ANGLE_INI_POS        40 //en esta posicion esta el angulo 181+40=221
#define ANGLE_END_POS        280 //en esta posicion esta el angulo 180-40=140

#define VALID_SCAN_SIZE      280

#define PRINT_PERIOD_MS      1000

#define MIN_CONFIDENCE       5

#define WHEEL_BASE_M        0.170f

//el tiempo total de giro para que le de tiempo a girar 180 grados
#define TOTAL_GIRO_180      20000

// Si un punto lleva más de esto sin actualizarse, lo imprimimos como 0.
// Para depurar puedes subirlo a 2000 o 3000.
#define POINT_MAX_AGE_MS     1500

#define CONFIG_TOPIC    "robot/config/sumo"


// =============================================================================
// Static state
// =============================================================================

/*
    Mapeo:

    index 0   -> 181º
    index 1   -> 182º
    ...
    index 178 -> 359º
    index 179 -> 0º
    index 180 -> 1º
    ...
    index 359 -> 180º
*/

static uint16_t s_lidar_scan_mm[LIDAR_SCAN_SIZE] = {0};
static uint8_t  s_lidar_scan_conf[LIDAR_SCAN_SIZE] = {0};
static uint32_t s_lidar_scan_ts_ms[LIDAR_SCAN_SIZE] = {0};

static volatile uint32_t s_packets_ok = 0;
static volatile uint32_t s_packets_bad = 0;
static volatile uint32_t s_points_ok = 0;
static volatile uint32_t s_bytes_rx = 0;

static TaskHandle_t s_lidar_rx_task_handle = NULL;
static TaskHandle_t s_lidar_print_task_handle = NULL;

static bool s_uart_initialized = false;

static portMUX_TYPE s_lidar_mux = portMUX_INITIALIZER_UNLOCKED;

static bool giro_180 = false;

static uint32_t s_giro_180_start_ms = 0;

// =============================================================================
// MQTT CONFIG
// =============================================================================

typedef struct {
    float kp_v;
    float kp_w;
    float max_v;
    float max_w;
    float base_v;
    float base_w;
    uint32_t tiempo_giro_180_ms;
} sumo_logic_config_t;

static sumo_logic_config_t s_current_config = {
    .kp_v = 0.01f, 
    .kp_w = 0.01f, 
    .max_v = 1.0f,
    .max_w = 1.0f,
    .base_v = 0.5f,
    .base_w = 0.5f,
    .tiempo_giro_180_ms = 2000
};

static void mqtt_config_callback(const char *topic, int topic_len, const char *data, int data_len) {
    if (data == NULL || data_len <= 0 || data_len > 1024) return;
    
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (root == NULL) return;

    cJSON *kp_v = cJSON_GetObjectItem(root, "kp_v");
    cJSON *kp_w = cJSON_GetObjectItem(root, "kp_w");
    cJSON *max_v = cJSON_GetObjectItem(root, "max_v");
    cJSON *max_w = cJSON_GetObjectItem(root, "max_w");
    cJSON *base_v = cJSON_GetObjectItem(root, "base_v");
    cJSON *base_w = cJSON_GetObjectItem(root, "base_w");
    cJSON *tiempo_giro_180_ms = cJSON_GetObjectItem(root, "tiempo_giro_180_ms");

    if (kp_v) s_current_config.kp_v = kp_v->valuedouble;
    if (kp_w) s_current_config.kp_w = kp_w->valuedouble;
    if (max_v) s_current_config.max_v = max_v->valuedouble;
    if (max_w) s_current_config.max_w = max_w->valuedouble;
    if (base_v) s_current_config.base_v = base_v->valuedouble;
    if (base_w) s_current_config.base_w = base_w->valuedouble;
    if (cJSON_IsNumber(tiempo_giro_180_ms) && tiempo_giro_180_ms->valueint >= 0) {
        s_current_config.tiempo_giro_180_ms = (uint32_t)tiempo_giro_180_ms->valueint;
    }
    ESP_LOGI(TAG,
         "Dynamic Config Updated: kp_v=%.3f kp_w=%.3f max_v=%.3f max_w=%.3f base_v=%.3f base_w=%.3f tiempo_giro_180_ms=%" PRIu32,
         s_current_config.kp_v,
         s_current_config.kp_w,
         s_current_config.max_v,
         s_current_config.max_w,
         s_current_config.base_v,
         s_current_config.base_w,
         s_current_config.tiempo_giro_180_ms);



    cJSON_Delete(root);
}




// =============================================================================
// Helpers
// =============================================================================

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t now_ms_u32(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static int normalize_angle_int(int angle)
{
    while (angle >= 360) {
        angle -= 360;
    }

    while (angle < 0) {
        angle += 360;
    }

    return angle;
}

static int angle_to_centered_index(float angle_deg)
{
    int angle_i = (int)(angle_deg + 0.5f);
    angle_i = normalize_angle_int(angle_i);

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

    if (start_angle_raw >= 36000 || end_angle_raw >= 36000) {
        return false;
    }

    float start_angle = start_angle_raw / 100.0f;
    float end_angle = end_angle_raw / 100.0f;

    float diff = end_angle - start_angle;
    if (diff < 0.0f) {
        diff += 360.0f;
    }

    /*
        Un paquete normal ocupa pocos grados.
        Si sale una barbaridad, seguramente hemos perdido sincronía.
    */
    if (diff <= 0.0f || diff > 40.0f) {
        return false;
    }

    return true;
}

static void lidar_clear_scan(void)
{
    portENTER_CRITICAL(&s_lidar_mux);

    memset(s_lidar_scan_mm, 0, sizeof(s_lidar_scan_mm));
    memset(s_lidar_scan_conf, 0, sizeof(s_lidar_scan_conf));
    memset(s_lidar_scan_ts_ms, 0, sizeof(s_lidar_scan_ts_ms));

    s_packets_ok = 0;
    s_packets_bad = 0;
    s_points_ok = 0;
    s_bytes_rx = 0;

    portEXIT_CRITICAL(&s_lidar_mux);
}

static void lidar_update_scan_point(float angle_deg,
                                    uint16_t distance_mm,
                                    uint8_t confidence)
{
    if (distance_mm == 0) {
        return;
    }

    if (confidence < MIN_CONFIDENCE) {
        return;
    }

    int idx = angle_to_centered_index(angle_deg);

    if (idx < 0 || idx >= LIDAR_SCAN_SIZE) {
        return;
    }

    uint32_t t_ms = now_ms_u32();

    portENTER_CRITICAL(&s_lidar_mux);

    s_lidar_scan_mm[idx] = distance_mm;
    s_lidar_scan_conf[idx] = confidence;
    s_lidar_scan_ts_ms[idx] = t_ms;
    s_points_ok++;

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
        uint8_t confidence = packet[packet_idx + 2];

        float angle = start_angle_deg +
                      ((end_angle_for_interp - start_angle_deg) * (float)p) /
                      (float)(LIDAR_POINTS - 1);

        if (angle >= 360.0f) {
            angle -= 360.0f;
        }

        lidar_update_scan_point(angle, distance_mm, confidence);
    }
}

static void lidar_get_scan_copy(uint16_t out_dist[LIDAR_SCAN_SIZE],
                                uint8_t out_conf[LIDAR_SCAN_SIZE],
                                uint32_t out_ts[LIDAR_SCAN_SIZE],
                                uint32_t *out_packets_ok,
                                uint32_t *out_packets_bad,
                                uint32_t *out_points_ok,
                                uint32_t *out_bytes_rx)
{
    portENTER_CRITICAL(&s_lidar_mux);

    memcpy(out_dist, s_lidar_scan_mm, sizeof(s_lidar_scan_mm));
    memcpy(out_conf, s_lidar_scan_conf, sizeof(s_lidar_scan_conf));
    memcpy(out_ts, s_lidar_scan_ts_ms, sizeof(s_lidar_scan_ts_ms));

    if (out_packets_ok)  *out_packets_ok  = s_packets_ok;
    if (out_packets_bad) *out_packets_bad = s_packets_bad;
    if (out_points_ok)   *out_points_ok   = s_points_ok;
    if (out_bytes_rx)    *out_bytes_rx    = s_bytes_rx;

    portEXIT_CRITICAL(&s_lidar_mux);
}

// =============================================================================
// LiDAR RX task
// =============================================================================

static void lidar_rx_task(void *arg)
{
    (void)arg;

    uint8_t packet[LIDAR_PACKET_SIZE];
    int packet_pos = 0;

    ESP_LOGI(TAG, "LiDAR RX task started");

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

        portENTER_CRITICAL(&s_lidar_mux);
        s_bytes_rx += (uint32_t)len;
        portEXIT_CRITICAL(&s_lidar_mux);

        if (packet_pos == 0) {
            if (byte == LIDAR_HEADER) {
                packet[0] = byte;
                packet_pos = 1;
            }
            continue;
        }

        if (packet_pos == 1) {
            if (byte == LIDAR_VER_LEN) {
                packet[1] = byte;
                packet_pos = 2;
            } else if (byte == LIDAR_HEADER) {
                packet[0] = byte;
                packet_pos = 1;
            } else {
                packet_pos = 0;
            }
            continue;
        }

        packet[packet_pos] = byte;
        packet_pos++;

        if (packet_pos < LIDAR_PACKET_SIZE) {
            continue;
        }

        packet_pos = 0;

        if (!lidar_packet_basic_valid(packet)) {
            portENTER_CRITICAL(&s_lidar_mux);
            s_packets_bad++;
            portEXIT_CRITICAL(&s_lidar_mux);
            continue;
        }

        portENTER_CRITICAL(&s_lidar_mux);
        s_packets_ok++;
        portEXIT_CRITICAL(&s_lidar_mux);

        lidar_parse_packet_update_scan(packet);

        /*
            No imprimir aquí.
            Esta tarea debe leer UART lo más rápido posible.
        */
    }
}

// =============================================================================
// LiDAR print task
// =============================================================================

static void lidar_print_task(void *arg)
{
    (void)arg;

    uint16_t dist[LIDAR_SCAN_SIZE];
    uint8_t conf[LIDAR_SCAN_SIZE];
    uint32_t ts[LIDAR_SCAN_SIZE];

    ESP_LOGI(TAG, "LiDAR print task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));

        uint32_t current_ms = now_ms_u32();

        uint32_t packets_ok = 0;
        uint32_t packets_bad = 0;
        uint32_t points_ok = 0;
        uint32_t bytes_rx = 0;

        lidar_get_scan_copy(
            dist,
            conf,
            ts,
            &packets_ok,
            &packets_bad,
            &points_ok,
            &bytes_rx
        );

        uint16_t nonzero_count = 0;

        for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
            if (dist[i] > 0 && ts[i] > 0 && (current_ms - ts[i]) <= POINT_MAX_AGE_MS) {
                nonzero_count++;
            }
        }

        printf("\n========== LIDAR 360 ARRAY ==========\n");
        printf("packets_ok=%lu packets_bad=%lu points_ok=%lu bytes_rx=%lu nonzero_angles=%u\n",
               (unsigned long)packets_ok,
               (unsigned long)packets_bad,
               (unsigned long)points_ok,
               (unsigned long)bytes_rx,
               nonzero_count);

        /*
            Array completo en una sola línea.
            Orden:
            [181, 182, ..., 359, 0, 1, ..., 180]
        */
        // printf("angles=[");
        // for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
        //     int angle = centered_index_to_angle(i);
        //     printf("%d", angle);
        //     if (i < LIDAR_SCAN_SIZE - 1) {
        //         printf(",");
        //     }
        // }
        // printf("]\n");

        printf("dist_mm=[");

        for (int i = 0; i < LIDAR_SCAN_SIZE; i++) {
            uint16_t value = 0;

            if (dist[i] > 0 && ts[i] > 0 && (current_ms - ts[i]) <= POINT_MAX_AGE_MS) {
                value = dist[i];
            }

            printf("%u\n", value);

            if (i < LIDAR_SCAN_SIZE - 1) {
                printf(",");
            }
        }

        printf("]\n");
        printf("=====================================\n");
    }
}

// =============================================================================
// UART init / task start-stop
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

    esp_err_t err = ESP_OK;

    err = uart_driver_install(
        LIDAR_UART_PORT,
        8192,
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

static void lidar_tasks_start(void)
{
    if (s_lidar_rx_task_handle == NULL) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            lidar_rx_task,
            "lidar_rx_task",
            4096,
            NULL,
            4,
            &s_lidar_rx_task_handle,
            1
        );

        if (ok != pdPASS) {
            ESP_LOGE(TAG, "No se pudo crear lidar_rx_task");
            s_lidar_rx_task_handle = NULL;
        }
    }

    //solo para debug, no imprimir en cada ejecución normal del modo sumo
    if (s_lidar_print_task_handle == NULL) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            lidar_print_task,
            "lidar_print_task",
            8192,
            NULL,
            1,
            &s_lidar_print_task_handle,
            1
        );

        if (ok != pdPASS) {
            ESP_LOGE(TAG, "No se pudo crear lidar_print_task");
            s_lidar_print_task_handle = NULL;
        }
    }
}

static void lidar_tasks_stop(void)
{
    if (s_lidar_rx_task_handle != NULL) {
        TaskHandle_t task = s_lidar_rx_task_handle;
        s_lidar_rx_task_handle = NULL;
        vTaskDelete(task);
    }

    if (s_lidar_print_task_handle != NULL) {
        TaskHandle_t task = s_lidar_print_task_handle;
        s_lidar_print_task_handle = NULL;
        vTaskDelete(task);
    }
}


// =============================================================================
// LOGICA SUMO
// =============================================================================

void find_closest_object(uint16_t array_lidar[], uint16_t* pos_object, uint16_t* dist_object){
    uint16_t pos_ini_objeto;
    uint16_t pos_end_objeto;
    *dist_object=1000;

    for(int i=0;i<VALID_SCAN_SIZE;i++){
        if(array_lidar[i]<*dist_object && array_lidar[i]>0){  //si el punto es más cercano que el más cercano encontrado hasta ahora, y es un punto válido (distancia > 0)
            pos_ini_objeto=i;
            while(i<VALID_SCAN_SIZE && array_lidar[i]<1000){
                i++;
            }
            pos_end_objeto=i;
            *pos_object=(pos_ini_objeto+pos_end_objeto)/2;
            *dist_object = array_lidar[*pos_object];
        }
    }
}


void sumo(float* vL, float* vR){
      
    uint16_t dist[LIDAR_SCAN_SIZE];
    uint8_t conf[LIDAR_SCAN_SIZE];
    uint32_t ts[LIDAR_SCAN_SIZE];
    uint32_t packets_ok = 0;
    uint32_t packets_bad = 0;
    uint32_t points_ok = 0;
    uint32_t bytes_rx = 0;

    lidar_get_scan_copy(
        dist,
        conf,
        ts,
        &packets_ok,
        &packets_bad,
        &points_ok,
        &bytes_rx
    );
    
    //Descarto la parte trasera del lidar, ya que esta el robot, y me quedo con los puntos desde ANGLE_INI_POS hasta ANGLE_END
    uint16_t array_lidar[VALID_SCAN_SIZE];
    uint16_t pos_real_array=0;

    for(int i=0;i<VALID_SCAN_SIZE;i++){
        if(i>=ANGLE_INI_POS && i<=ANGLE_END_POS){
            array_lidar[pos_real_array]=dist[i];
            pos_real_array++;
        }
    }

    uint16_t pos_object=0;
    uint16_t dist_object=0;
    float v=0;
    float w=0;

    find_closest_object(array_lidar,&pos_object,&dist_object);

    if(pos_object==0 && dist_object==0){    //no he encontrado el objeto, por lo que giro a la izquierda
        v=0;
        w=s_current_config.base_w;
        *vL = v-(w*WHEEL_BASE_M/2.0f);
        *vR = v+(w*WHEEL_BASE_M/2.0f);
        return;
    }

    uint8_t pos_centro = VALID_SCAN_SIZE/2;
    int8_t dif_centro = pos_object-pos_centro;  

    if(dif_centro>-5&&dif_centro<5){
        dif_centro=1;
    }else{
        w = s_current_config.base_w + s_current_config.kp_w*dif_centro; //no hace falta mirar si es izquierda o derecha porque ya lo dice el signo
        if(w>s_current_config.max_w){
            w=s_current_config.max_w;
        }
    }

    v = s_current_config.base_v + s_current_config.kp_v*dist_object*(1.0f/abs(dif_centro));
    if(v>s_current_config.max_v){
        v=s_current_config.max_v;
    }

    *vL = v-(w*WHEEL_BASE_M/2.0f);
    *vR = v+(w*WHEEL_BASE_M/2.0f);
    
}

// =============================================================================
// Mode callbacks
// =============================================================================

static void enter(void)
{
    ESP_LOGI(TAG, "Entering SUMO LiDAR test mode");

    lidar_clear_scan();
    mqtt_custom_client_register_topic_callback(CONFIG_TOPIC,    mqtt_config_callback);
    if (lidar_uart_start() == ESP_OK) {
        lidar_tasks_start();
    }
}

static void execute(motor_driver_mcpwm_t* motors,
                    motor_velocity_ctrl_handle_t ctrl_left,
                    motor_velocity_ctrl_handle_t ctrl_right,
                    float dt_s)
{

    (void)dt_s;

    shared_memory_t* shm = shared_memory_get();
    
    // 1. Read Inputs (TODO NATIVO EN METROS Y METROS/SEGUNDO)
    xSemaphoreTake(shm->mutex, portMAX_DELAY);
    bool detected = shm->sensors.line_detected;
    float cur_l = shm->sensors.motor_speed_left;
    float cur_r = -shm->sensors.motor_speed_right;
    float bat_mv = shm->sensors.battery_voltage;
    xSemaphoreGive(shm->mutex);

    if (detected) {
        giro_180=true;
        s_giro_180_start_ms = now_ms_u32();
    }

    float vL=0;
    float vR=0;

    if(giro_180){
        if(now_ms_u32()-s_giro_180_start_ms>=s_current_config.tiempo_giro_180_ms){
            giro_180=false;
            contador_giro_180=0;
            vL = 0;
            vR = 0;
        }else{
            vL=-1;
            vR=1;
            contador_giro_180++;
        }
    }else{
        sumo(&vL,&vR);
    }

    motor_velocity_input_t motor_l = { .target_speed = vL, .current_speed = cur_l, .battery_mv = bat_mv };
    motor_velocity_input_t motor_r = { .target_speed = vR, .current_speed = cur_r, .battery_mv = bat_mv };

    float pwm_l, pwm_r;
    motor_velocity_ctrl_update(ctrl_left,  &motor_l, dt_s, &pwm_l, NULL);
    motor_velocity_ctrl_update(ctrl_right, &motor_r, dt_s, &pwm_r, NULL);

    motor_mcpwm_set(motors, (int16_t)(pwm_l * 10.0f), (int16_t)(pwm_r * 10.0f));
}

static void exit_mode(motor_driver_mcpwm_t* motors)
{
    ESP_LOGI(TAG, "Exiting SUMO LiDAR test mode");

    lidar_tasks_stop();

    motor_mcpwm_stop(motors);
}

const mode_interface_t mode_sumo = {
    .enter = enter,
    .execute = execute,
    .exit = exit_mode
};