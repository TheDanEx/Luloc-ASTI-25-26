# Telemetry Manager

## Propósito Arquitectónico
Abstrae la construcción de mensajes de telemetría estructurados siguiendo el formato **Influx Line Protocol (ILP)**. Actúa como un motor de acumulación (*Batching*) en SRAM para recolectar métricas de alta frecuencia y emitirlas periódicamente por MQTT, optimizando el ancho de banda y reduciendo la carga del broker.

## Entorno y Dependencias
Depende del wrapper `mqtt_custom_client.h` para el transporte y de `esp_timer.h` para los timestamps de precisión. Se ejecuta en una tarea dedicada dentro del **CPU1**.

## Interfaces de E/S (Inputs/Outputs)
- **Inputs:** Funciones `telemetry_add_*` para inyectar campos y `telemetry_commit_point()` para cerrar una lectura (fila) y añadirla al buffer de salida.
- **Outputs:** Strings puros en formato ILP transmitidos por MQTT al tópico configurado.

## Flujo de Ejecución Lógico
1. El usuario crea un reportero persistente con `telemetry_create()`.
2. Se pueden definir etiquetas estáticas con `telemetry_set_tags()` (ej. `sensor=battery`).
3. En cada lectura de sensor, se añaden campos (`add_float`, etc) y se llama a `telemetry_commit_point()`. Esto añade una línea con timestamp en nanosegundos (`esp_timer_get_time() * 1000`) al buffer interno.
4. Internamente, una tarea FreeRTOS despierta cada `interval_ms`, publica todo el buffer acumulado (varias líneas separadas por `\n`) y limpia el buffer.

## Funciones Principales y Parámetros
- `telemetry_create(topic, measurement, interval_ms)`: Inicializa el reportero y su tarea de publicación.
- `telemetry_set_tags(handle, tags)`: Define etiquetas fijas (ej. `"sensor=battery"`) que irán en cada línea.
- `telemetry_add_float(handle, key, value)`: Añade un campo decimal al punto actual.
- `telemetry_commit_point(handle)`: Finaliza la lectura actual, le adosa el timestamp en nanosegundos y la guarda en el buffer de SRAM.
- `telemetry_destroy(handle)`: Finaliza la tarea, libera los buffers y el handle.

## Puntos Críticos y Depuración
- **Saturación del Buffer (SRAM Batching):** El buffer tiene un tamaño fijo (`MAX_BUFFER_SIZE = 2048`). Si se acumulan demasiados puntos antes de que venza el `interval_ms`, los nuevos puntos se descartarán para evitar desbordamientos. 
- **Timestamps:** Se utilizan nanosegundos basados en el temporizador del hardware. Estos son relativos al arranque a menos que se sincronice el sistema mediante NTP.

## Ejemplo de Uso e Instanciación
```c
#include "telemetry_manager.h"

// 1. Inicialización (una sola vez)
telemetry_handle_t tel = telemetry_create("robot/telemetry/power", "power_system", 1000);
telemetry_set_tags(tel, "sensor=battery");

// 2. Bucle de lectura (Alta frecuencia)
while(1) {
    float v = read_voltage();
    telemetry_add_float(tel, "voltage_mv", v);
    telemetry_commit_point(tel); // Guarda el punto con su nano-timestamp
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
}

// El manager enviará el batch de 10 lecturas cada 1000ms automáticamente.
```
