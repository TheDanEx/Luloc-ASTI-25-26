# Application Main (app_main)

## Propósito Arquitectónico
El verdadero "Punto de ignición" (Entry-point) del C en la arquitectura ESP-IDF moderna. Un pequeño andamio logístico temporal que funge como maestro de ceremonias orquestando inicializaciones antes de que la rutina de Scheduler final arranque y despache a un segundo plano.

## Entorno y Dependencias
Corre bajo el sub-proceso nativo de la BIOS del ESP-IDF (Task "Main" en Core 0, por lo regular). Depende abstractamente de los archivos de tareas (`task_*_start()`) y de las rutinas agnósticas en `system_init()`.

## Interfaces de E/S (Inputs/Outputs)
- **Software:** Puramente invocador. Ejecuta rutinas estáticas sin registrar variables largas ni handles locales más allá de los logs formatados de `[ OK ]`.

## Flujo de Ejecución Lógico
La función `app_main(void)` invoca a `system_init()` (Que bloquea NVS/Red/Sonido/Buses). Subsiguientemente, va lanzando hilos esclavos "Freerunning" (Tareas RTOS) uno por uno a sus respectivos sub-núcleos (CPU1 y CPU0). Incorpora pequeños `vTaskDelay` mecánicos (100ms) preventivos entre llamadas de tareas agresivas para dar espacio a context-switches limpios o inicializaciones internas de pila.

## Funciones Principales y Parámetros
- `app_main(void)`: Declaración fundacional de ESP-IDF. Destruye sí mismo o permite retorno al terminar su ejecución lineal, debido a que FreeRTOS ya ha tomado los rieles del robot mediante Tasks, asumiendo su propio rol inactivo.

## Puntos Críticos y Depuración
- **Deadlocks Semánticos de Inicio:** Si dentro del código se llamara cualquier rutina de sondeo antes de las tareas completadas (`task_*/start`) que asumiera red viva o WiFi levantada con Mutex Infinitos, este archivo se detendría, evitando que nunca nazcan los nodos PIDs ni las mitigaciones de emergencia de potencia (El robot quedaría paralizado en boot).
