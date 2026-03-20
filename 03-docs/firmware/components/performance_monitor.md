# Performance Monitor

## Propósito Arquitectónico
Módulo observador que supervisa proactivamente el consumo general de CPU en todos los núcleos. Mide además la "Health" de la memoria Heap (libre histórica) y variables críticas como tiempos de bajo consumo, reportándolo estadísticamente para ser usado indirectamente en telemetría en tiempo real o logs.

## Entorno y Dependencias
Basado en llamadas directas al kernel de FreeRTOS (probablemente habilitando `vTaskGetRunTimeStats` o hooks crudos del framework de ESP-IDF para mediciones de trazas y timers de hardware absolutos de microsegundos).

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Tiempos de Halt/Sleep físicos medidos.
- **Software:** Estructuras absolutas transparentes al usuario accesibles mediante `perf_mon_get_stats_absolute()`, pre-formateo **Influx Line Protocol (ILP)** expuesto como buffer (`perf_mon_get_report_ilp()`) para tareas esclavas, devolviendo métricas de uso fraccionado con marcas de tiempo en nanosegundos.

## Flujo de Ejecución Lógico
Tras `perf_mon_init()`, debe ser explícitamente disparado a ritmo conservador llamando a `perf_mon_update()` (ej. cada 5000ms). Durante el update se capturan los snapshots de CPU, descartando matemáticamente los tiempos de las tareas Idle de cada CPU para ofrecer un porcentaje realista de ocupamiento global. Luego, llamadas get devuelven esto al llamador (típicamente `task_comms`).

## Funciones Principales y Parámetros
- `perf_mon_init(void)`: Configura sub-estructuras internas y rutinas necesarias de medición sin parámetros.
- `perf_mon_update(void)`: Toma un *snapshot* del tiempo consumido por RTOS y memoria Heap actual. Idealmente se llama periocamente desde un task lento (ej. cada 2 a 5 segundos).
- `perf_mon_get_stats_absolute(perf_data_abs_t *stats)`: Devuelve los datos puros y acumulativos del sistema.
  - `stats`: Puntero a estructura `perf_data_abs_t` que contendrá ticks totales de CPU0, CPU1, Heap libre e Idle.
- `perf_mon_get_report_ilp(char *buffer, size_t max_len, int64_t timestamp_ns)`: Genera un resumen en formato Influx Line Protocol listo para ser enviado por MQTT.
  - `buffer`: Puntero al arreglo de texto vacío.
  - `max_len`: Tamaño máximo del buffer preventivo del desbordamiento.
  - `timestamp_ns`: Marca de tiempo en nanosegundos (19 dígitos).
- `perf_mon_get_task_info_json(char *buffer, size_t max_len)`: Emite JSON profundo listando cada tarea (task) RTOS iterada en vivo con su ocupación.

## Puntos Críticos y Depuración
- **Sobrecarga de Scheduler:** Si `perf_mon_update()` es llamado frenéticamente desde un loop menor a 1 segundo, la congelación momentánea del contexto durante el barrido de tareas (si usa `vTaskGetRunTimeStats`) causará latencia errática afectando bucles de control de movimiento u observadores PID. 

## Ejemplo de Uso e Instanciación
```c
#include "performance_monitor.h"
#include "esp_log.h"

// 1. Tarea dedicada a diagnósticos (baja prioridad, ej. Core 0)
void diagnostics_task(void *pvParameters) {
    // Inicializa estructuras y timers internos (solo una vez)
    perf_mon_init();

    char ilp_buffer[512];

    while(1) {
        // Bloquear 5 segundos. Nunca hacer polling abusivo.
        vTaskDelay(pdMS_TO_TICKS(5000));

        // 2. Ejecutar muestreo profundo de RTOS y Memoria
        perf_mon_update();

        // 3. Imprimir a consola o enviar por MQTT (formato ILP)
        if (perf_mon_get_report_ilp(ilp_buffer, sizeof(ilp_buffer), esp_timer_get_time() * 1000) == ESP_OK) {
            ESP_LOGI("PERF", "ILP Metrics Generated:\n%s", ilp_buffer);
            // Salida típica: cpu_usage,task=IDLE0,core=0,robot=lurloc usage=99.2 1710892800000000000
        }
    }
}
```
