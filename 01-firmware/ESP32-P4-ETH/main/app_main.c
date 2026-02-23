/* ESP32-P4 Robot Controller with Ethernet and MQTT

   Combines Ethernet connectivity with MQTT for remote control
*/

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "system_init.h"
#include "task_rtcontrol_cpu0.h"
#include "task_comms_cpu1.h"
#include "task_monitor_lowpower_cpu1.h"
Servo transpaleta;



static const char *TAG = "APP_SERVO";

/**
 * @brief Control de servo estándar 0-180° conectado a GPIO32
 *
 * Frecuencia: 50Hz, pulsos 1000-2000µs
 */

void app_main(void)
{
    // ========== SYSTEM INITIALIZATION ==========
    // Handles NVS, Netif, Ethernet, Audio, Logging config, and Shared Memory
    system_init();
    
    // ========== BASIC INFO ==========
    // ========== START TASKS ==========
    
    // 1. Communications Task (Core 1)
    task_comms_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Comms Task");
    
    // Allow brief stabilization
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 2. Real-Time Control Task (Core 0)
    task_rtcontrol_cpu0_start();
    printf("[CPU%d] %-40s [ OK ]\n", 0, "Started RT Control");

    // 3. Low-Power Monitor Task (Core 1)
    task_monitor_lowpower_cpu1_start();
    printf("[CPU%d] %-40s [ OK ]\n", 1, "Started Monitor Task");

    ESP_LOGI(TAG, "=== Control de Servo 0-180° ===");
    ESP_LOGI(TAG, "Inicializando servo en GPIO 32 (0-180°)...");

    // Inicializar servo estándar (0-180 grados)
    Servo servo_motor;
    servo_init(&servo_motor, 32, 0.0f, 180.0f, SERVO_MODE_STANDARD);

    ESP_LOGI(TAG, "Servo inicializado en GPIO32. Iniciando secuencia de posiciones.");

    servo_init(&transpaleta, 18, 0.0, 1.0, SERVO_MODE_CONTINUOUS);

    // Calibrar bajando completamente
    servo_set_speed(&transpaleta, -1);
    vTaskDelay(pdMS_TO_TICKS(2500));
    servo_set_speed(&transpaleta, 0);
    transpaleta.current_pos = 0.0f;

    // Subir a la mitad
    servo_set_normalized(&transpaleta, 0.5);

    // Subir al máximo
    servo_set_normalized(&transpaleta, 1.0);

    // Bajar al mínimo
    servo_set_normalized(&transpaleta, 0.0);
}

    

