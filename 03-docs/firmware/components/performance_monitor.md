# Performance Monitor

## Propósito Arquitectónico
Módulo observador que supervisa proactivamente el consumo general de CPU en todos los núcleos. Mide además la "Health" de la memoria Heap (libre histórica) y variables críticas como tiempos de bajo consumo, reportándolo estadísticamente para ser usado indirectamente en telemetría en tiempo real o logs.

## Entorno y Dependencias
Basado en llamadas directas al kernel de FreeRTOS (probablemente habilitando `vTaskGetRunTimeStats` o hooks crudos del framework de ESP-IDF para mediciones de trazas y timers de hardware absolutos de microsegundos).

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Tiempos de Halt/Sleep físicos medidos.
- **Software:** Estructuras absolutas transparentes al usuario accesibles mediante `perf_mon_get_stats_absolute()`, pre-formateo JSON expuesto como buffer (`perf_mon_get_report_json()`) para tareas esclavas, devolviendo Arrays de uso fraccionado.

## Flujo de Ejecución Lógico
Tras `perf_mon_init()`, debe ser explícitamente disparado a ritmo conservador llamando a `perf_mon_update()` (ej. cada 5000ms). Durante el update se capturan los snapshots de CPU, descartando matemáticamente los tiempos de las tareas Idle de cada CPU para ofrecer un porcentaje realista de ocupamiento global. Luego, llamadas get devuelven esto al llamador (típicamente `task_comms`).

## Funciones Principales y Parámetros
- `perf_mon_init(void)`: Configura sub-estructuras internas y rutinas necesarias de medición sin parámetros.
- `perf_mon_update(void)`: Toma un *snapshot* del tiempo consumido por RTOS y memoria Heap actual. Idealmente se llama periocamente desde un task lento (ej. cada 2 a 5 segundos).
- `perf_mon_get_stats_absolute(perf_data_abs_t *stats)`: Devuelve los datos puros y acumulativos del sistema.
  - `stats`: Puntero a estructura `perf_data_abs_t` que contendrá ticks totales de CPU0, CPU1, Heap libre e Idle.
- `perf_mon_get_report_json(char *buffer, size_t max_len)`: Genera un resumen humanizable formateado listo para red.
  - `buffer`: Puntero al arreglo de texto vacío.
  - `max_len`: Tamaño máximo del buffer preventivo del desbordamiento.
- `perf_mon_get_task_info_json(char *buffer, size_t max_len)`: Emite JSON profundo listando cada tarea (task) RTOS iterada en vivo con su ocupación.

## Puntos Críticos y Depuración
- **Sobrecarga de Scheduler:** Si `perf_mon_update()` es llamado frenéticamente desde un loop menor a 1 segundo, la congelación momentánea del contexto durante el barrido de tareas (si usa `vTaskGetRunTimeStats`) causará latencia errática afectando bucles de control de movimiento u observadores PID. 
