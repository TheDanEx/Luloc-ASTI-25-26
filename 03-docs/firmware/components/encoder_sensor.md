# Encoder Sensor

## Propósito Arquitectónico
Módulo encargado de la lectura física de odometría de los motores a través de encoders de cuadratura. Su propósito principal es aislar la lógica compleja del cálculo de velocidad lineal y distancia desplazada basándose en las constantes mecánicas de la rueda.

## Entorno y Dependencias
Emplea la API moderna `driver/pulse_cnt.h` de ESP-IDF (PCNT - Pulse Counter) para no cargar la CPU contando flancos por interrupciones manuales por software.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Dos pines GPIO digitales por sensor que leen impulsos del canal A y el canal B.
- **Software:** Un *handle* opaco que ofrece acceso constante para consultar mediante polling variables críticas en formato real (`float`): velocidad (`m/s`) y distancia absoluta/relativa.

## Flujo de Ejecución Lógico
Requiere paso previo de inicialización mediante `encoder_sensor_config_t` donde se especifican pulsos por vuelta (PPR), relación de reducción de caja (Gear ratio) y tamaño de rueda. 
Se consulta cíclicamente (típicamente desde un bucle principal de control) con `encoder_sensor_get_speed()` para el lazo cerrado de control.

## Funciones Principales y Parámetros
- `encoder_sensor_init(const encoder_sensor_config_t *config)`: Inicializa el sensor del encoder.
  - `config`: Estructura con la configuración (pines A/B, PPR, ratio de reducción, diámetro de la rueda y flag de reversa).
  - Devuelve un *handle* opaco (`encoder_sensor_handle_t`) para interactuar con este encoder.
- `encoder_sensor_get_speed(encoder_sensor_handle_t handle)`: Obtiene la velocidad lineal actual.
  - `handle`: El puntero del encoder previamente inicializado.
  - Devuelve la velocidad en float (metros/segundo).
- `encoder_sensor_get_distance(encoder_sensor_handle_t handle)`: Obtiene la distancia total recorrida.
  - `handle`: Puntero del encoder.
  - Devuelve la distancia relativa acumulada en float (metros).
- `encoder_sensor_reset_distance(encoder_sensor_handle_t handle)`: Pone el contador de distancia a cero.
  - `handle`: Puntero del encoder a reiniciar.

## Puntos Críticos y Depuración
- **Pérdida de pulsos:** A velocidades muy altas, los glitches o ruido eléctrico pueden falsear datos del PCNT, arruinando el PID o SLAM. Considerar filtros Glitch por hardware de ESP32.
- **Desbordamiento numérico (Overflow):** Si el contador base llega a su límite, debe ser reseteado transparentemente o afectará severamente los sumatorios de distancia recorrida.

## Ejemplo de Uso e Instanciación
```c
#include "encoder_sensor.h"
#include "driver/gpio.h"

// 1. Configuración de parámetros físicos del encoder
encoder_sensor_config_t config_rueda_izq = {
    .pin_a = GPIO_NUM_4,          // Canal A
    .pin_b = GPIO_NUM_5,          // Canal B
    .ppr = 360,                   // Pulsos por revolución pura del encoder
    .gear_ratio = 34.0f,          // Relación de la caja reductora (ej 34:1)
    .wheel_diameter_m = 0.065f,   // Diámetro físico de la rueda en metros
    .reverse = false              // Sentido de giro directo
};

// 2. Inicialización en la Tarea de Control (ej. task_comms_cpu1.c)
encoder_sensor_handle_t encoder_izquierdo;

void main_task(void *pvParameters) {
    encoder_izquierdo = encoder_sensor_init(&config_rueda_izq);

    while (1) {
        // 3. Extracción de Telemetría pura a ~100Hz
        float vel_lineal = encoder_sensor_get_speed(encoder_izquierdo);
        float odometria_total = encoder_sensor_get_distance(encoder_izquierdo);

        // Envío por Telemetría ILP o uso en PID...
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```
