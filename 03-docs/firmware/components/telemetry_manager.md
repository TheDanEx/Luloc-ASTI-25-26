# Telemetry Manager

## Propósito Arquitectónico
Abstrae la construcción de mensajes de telemetría estructurados. Actúa como un builder para recolectar diferentes datos (flotantes, enteros, booleanos) de los sensores en tiempo real y prepararlos para ser enviados por red en un formato que bases de datos o brokers entiendan nativamente (JSON/Influx).

## Entorno y Dependencias
Depende estrictamente del wrapper inferior `mqtt_custom_client.h`. Pensado para entornos donde el stack TCP/IP y la sesión MQTT ya están inicializadas.

## Interfaces de E/S (Inputs/Outputs)
- **Software:** Constructor que devuelve un puntero opaco `telemetry_handle_t` configurado con métricas principales (Topic y Measurement ID e Intervalos recomendados). Funciones inyectoras `telemetry_add_float()`, `telemetry_add_int()`, `telemetry_add_bool()`.

## Flujo de Ejecución Lógico
Al iniciar la comunicación o telemetría, el programa (CPU1) crea instancias de "reporteros" mediante `telemetry_create()`. Durante el ciclo útil se agrupan campos llamando iterativamente a los métodos `add_*`. Al final de la rutina y dependiendo del flujo de código `.c`, el sistema emite todo el diccionario por el puerto MQTT asociado y reinicia su estado.

## Funciones Principales y Parámetros
- `telemetry_create(const char *topic, const char *measurement, uint32_t interval_ms)`: Funda un builder de reporte telemétrico temporal pre-etiquetado para bases de datos e inicializa la memoria dinámica de su string.
  - `topic`: Ruta MQTT destino (`robot/telemetry`).
  - `measurement`: Etiqueta principal JSON. Retorna handle opaco `telemetry_handle_t`.
- `telemetry_add_float(telemetry_handle_t handle, const char *key, float value)`: Inyecta una métrica dentro del buffer de iteración actual. 
  - `key`: Etiqueta explícita de la variable (p. ej. `"velocity"`).
  - `value`: Número de tipo decimal de lectura.
- `telemetry_add_int(...)` / `telemetry_add_bool(...)`: Variantes polimórficos de agregado tipado a la cadena JSON.
- `telemetry_destroy(telemetry_handle_t handle)`: Cierra la trama completa del JSON actual, efectúa el trigger de comunicación por red hacia MQTT y finalmente OBLIGATORIAMENTE destruye y libera (Free) la memoria del string alojada para ese reporte, liquidando el handle.

## Puntos Críticos y Depuración
- **Memory Leaks Ocultos:** Si no se emplea correctamente la macro de destrucción `telemetry_destroy(handle)`, el montón (Heap) de FreeRTOS será agotado aceleradamente causando reinicios por OOM (Out Of Memory) impredecibles.
- **Saturación del Buffer Interno:** Asumiblemente existe un char buffer alojado detrás del handle. Si un desarrollador intenta incrustar docenas de variables float de precision muy amplia, superará el tamaño del buffer corrompiendo memoria RAM general (Buffer Overflow) a menos que esté estrictamente blindado internamente.

## Ejemplo de Uso e Instanciación
```c
#include "telemetry_manager.h"
#include "mqtt_custom_client.h"

// Tarea Cíclica de Reporte (Core 1) a 10Hz
void telemetry_sys_task(void *pvParameters) {
    while(1) {
        // 1. Iniciar la trama Influx Line Protocol (ILP) o JSON
        // Esto aloja dinámicamente un buffer en el Heap
        telemetry_handle_t frame = telemetry_create("robot/metrics", "kinematics", 100);

        if (frame != NULL) {
            // 2. Inyectar variables de forma transparente (El timestamp lo pone por debajo)
            telemetry_add_float(frame, "speed_ms", current_speed);
            telemetry_add_int(frame, "battery_pct", battery_level);
            telemetry_add_bool(frame, "is_autonomous", mode_auto);

            // 3. Empaquetar y Disparar el envío por red 
            // CUIDADO: Destruye el buffer OBLIGATORIAMENTE evitando fugas de memoria
            telemetry_destroy(frame);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar 100ms
    }
}
```
