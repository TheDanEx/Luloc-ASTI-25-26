# Line Sensor Array

## Propósito Arquitectónico
Proporciona una interfaz de alto rendimiento, no bloqueante, para leer arrays de sensores de línea reflectivos (ej., matrices TCRT5000, sensores QTR). Utiliza la moderna API `adc_oneshot` de ESP-IDF para muestrear múltiples canales analógicos de forma simultánea.

Siguiendo las directrices de programación defensiva, el componente emplea un **Patrón de Puntero Opaco (Handle/Clase)**. Esto permite instanciar múltiples arrays de sensores independientes (por ejemplo, un array frontal y otro trasero) sin colisiones de estado ni accesos de memoria accidentales.

## Entorno y Dependencias
Depende de `driver/adc_oneshot.h` para lecturas analógicas crudas, `freertos/task.h` para el hilo de calibración asíncrono, y `esp_timer.h`. Utiliza Kconfig (`idf.py menuconfig`) para inyectar los umbrales de detección y tasas de sobremuestreo en tiempo de compilación.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Pines ruteables hacia unidades ADC genéricas del ESP32-P4.
- **Software:** 
  - `line_sensor_init()` recibe un listado tipado de canales ADC y distancias físicas.
  - Genera una salida estructurada `line_sensor_data_t` que incluye: mediciones analógicas crudas (`raw_values`), pesajes de 0.0 a 1.0 (`normalized_values`), estados lógicos pre-evaluados (`digital_states`), y la distancia física calculada del centroide (`line_position_m`).

## Flujo de Ejecución Lógico
La inicialización aloja toda la memoria interna y configura el Hardware ADC.
Al llamar a `line_sensor_calibration_start()`, se levanta una tarea FreeRTOS oculta en el Core 1. Esta tarea orbita a 200Hz escaneando los valores mínimos (superficie negra) y máximos (línea blanca) de *todos* los fotodiodos simultáneamente, sin bloquear el hilo principal.
Una vez calibrado, la invocación de `line_sensor_read()` efectúa un barrido rápido con sobremuestreo (oversampling), normaliza las variables respecto a los bounds de calibración, ejecuta un _thresholding_ digital, y computa el centro de masa métrico.

## Funciones Principales y Parámetros
- `line_sensor_init(const line_sensor_config_t *config)`: Reserva recursos, buffers asíncronos de flotantes, e inicializa el driver ADC nativo. Retorna `line_sensor_handle_t` opaco.
- `line_sensor_calibration_start(line_sensor_handle_t handle)`: Despliega el RTOS Task de calibración hiperrápida. El robot debe balancearse sobre la línea en este momento.
- `line_sensor_calibration_stop(line_sensor_handle_t handle)`: Destruye limpiamente el hilo de calibración interna.
- `line_sensor_read(line_sensor_handle_t handle, line_sensor_data_t *out_data)`: Unifica todo el procesado algorítmico y matemático (Ruido -> Normalización -> Digitalización -> Centroide). Escribe el resultado consolidado en el puntero de salida brindado.
  - El puntero `out_data` devuelve una estructura con 4 campos clave listos para el lazo de control PID:
    1. **`out_data->line_position_m`** `(float)`: Distancia exacta de la línea al centro del robot en metros (ej: `-0.015` = 1.5cm a la izquierda).
    2. **`out_data->line_detected`** `(bool)`: `true` si agún sensor de la matriz detecta superficie blanca. Si es `false`, el robot se ha salido completamente del circuito.
    3. **`out_data->digital_states`** `(bool*)`: Array indicando qué sensores específicos ven línea (ej: `if(out_data->digital_states[0] == true)` significa curva cerrada extrema izquierda).
    4. **`out_data->normalized_values`** `(float*)`: Pesos del 0.0 al 1.0 por si se desea correr un algoritmo de visión artificial propio sobre las sombras del circuito.
  - **Ejemplo de Uso Real (Control Loop):**
    ```c
    line_sensor_data_t sensor_data;
    if (line_sensor_read(my_sensor_handle, &sensor_data) == ESP_OK) {
        if (!sensor_data.line_detected) {
            // Caso especial: El robot descarriló totalmente. Frenar motores o rotar sobre si mismo.
            motor_mcpwm_brake(my_motors);
        } else if (sensor_data.digital_states[0] && sensor_data.digital_states[7]) {
            // Caso especial: Intersección cruzada detectada (sensores extremos ambos activos).
            navigate_intersection();
        } else {
            // Seguimiento normal: inyectar el error en metros directamente al cálculo PID.
            float error_m = 0.0f - sensor_data.line_position_m;
            pid_compute(error_m);
        }
    }
    ```
- `line_sensor_read_raw(line_sensor_handle_t handle, uint16_t *out_raw)`: (Función debug) Permite extraer la matriz primitiva del ADC sin procesamiento adicional.

## Puntos Críticos y Depuración
- **Deadlocks por Multiplexado:** Las lecturas y la calibración asíncrona compiten por el mismo periférico ADC. Se requiere y asume la protección inquebrantable impuesta mediante el `SemaphoreHandle_t mutex` en todas las APIs del componente.
- **División por Cero (Control):** Durante el arranque en frío o si el robot es levantado del suelo sin calibrar (Rango Max y Min idénticos), el algoritmo de normalización inyecta salvaguardas (clipping `0.0f` a `1.0f`) para prevenir que un `NaN` corrompa el lazo de los controladores PID aguas arriba.

## Ejemplo de Uso e Instanciación
```c
#include "line_sensor.h"
#include "hal/adc_types.h"

// 1. Array de Canales Hardware ADC
const adc_channel_t pines_frontales[] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
};

// 2. Distancias físicas relativas al centro del robot (en metros)
const float distancias_m[] = {
    -0.05f, -0.035f, -0.02f, -0.005f, 0.005f, 0.02f, 0.035f, 0.05f
};

// 3. Declaración de Módulo
line_sensor_config_t config_frontal = {
    .num_sensors = 8,
    .adc_unit = ADC_UNIT_1,
    .adc_channels = pines_frontales,
    .sensor_positions_m = distancias_m,
    .oversample_count = 0,        // = Toma valor default Kconfig
    .calibration_threshold = 0,   // = Toma valor default Kconfig
    .detection_threshold = 0.0f   // = Toma valor default Kconfig
};

// Inicialización
line_sensor_handle_t array_frontal = line_sensor_init(&config_frontal);

// Iniciar Calibración (mover el robot de lado a lado)
line_sensor_calibration_start(array_frontal);

// ... Más tarde en tu lazo PID (a más de 100Hz) ...
if (line_sensor_is_calibrated(array_frontal)) {
    line_sensor_data_t datos;
    if (line_sensor_read(array_frontal, &datos) == ESP_OK) {
        if (!datos.line_detected) {
            motor_mcpwm_brake(mis_motores); // ¡Salida de Pista Total!
        } else {
            // Error métrico exacto respecto al centro del robot (setpoint = 0.0m)
            float error_distancia_m = 0.0f - datos.line_position_m;
            float correccion = compute_pid(error_distancia_m);
        }
    }
}
```
