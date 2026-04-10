# Task Real-Time Control (task_rtcontrol_cpu0)

## Propósito Arquitectónico
Cerebro de actuación en tiempo reall (RT) puro del Lurloc-ASTI. Diseñado explícita y únicamente para ejecutar en lazo cerrado cálculos intensivos como PIDs de rueda cruzada, evasión micro-refleja de obstáculos, control balístico base y manipulación de los generadores PWM de los Drivers, totalmente a salvo de desincronizaciones del mundo exterior IP.

## Entorno y Dependencias
- Tarea reina asignada férreamente al **Core 0** (núcleo PRO) con **Prioridad 10** de FreeRTOS.
- Posee integración exclusiva de acoplamiento hardware directa con `motor.h` (Generadores MCPWM de alta cadencia).

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Orquestador de los pines PWM Lógicos conectados a los H-Bridges (GPIO 25/26 izquierdo, 23/5 derecho).
- **Software:** De momento un andamio autónomo de demo. Eventualmente se apalancará de `shared_memory.h` para sacar sus variables PID internas e inocular los comandos asíncronos en comandos vectoriales síncronos.

## Flujo de Ejecución Lógico
Init crudo del MCPWM Timer general. En el bucle principal infinito, efectúa sub-llamadas asíncronas hiper-rápidas a las primitivas base de actualización del comparador PWM (`motor_mcpwm_set`), seguido de retardos medidos, aplicando secuencias (Adelante, Parada, Atras) secuencial o basado en callbacks en un futuro de 100Hz o más.

## Funciones Principales y Parámetros
- `task_rtcontrol_cpu0_start(void)`: Arranca la pila alojando 4096 bytes con fuerte directiva `xTaskCreatePinnedToCore` forzando ubicación en `CORE_0`.

## Puntos Críticos y Depuración
- **Apropiación del Núcleo Crítica (Core Hoarding):** Jamás este bucle debe bloquear al procesador. El FreeRTOS ha sido reconfigurado forzosamente a **1000Hz (`CONFIG_FREERTOS_HZ=1000`)** en su `sdkconfig` para permitir escalares de retardo asíncronos asfixiantes como `2ms (500 Hz)`, controlados por la variable variable del usuario de `CONFIG_ROBOT_CONTROL_PERIOD_MS`. Si el OS decae a 100Hz o hay errores en el `pdMS_TO_TICKS()`, saltará un crasheo del Watchdog `IDLE0 Task watchdog got triggered`.
- **Riesgos de Punteros IPC:** Una vez use `shared_memory`, si olvida envolver la asignación interna con Mutex, las velocidades asíncronas corromperán la flotación temporal de la memoria causando estirones o bloqueos transitorios (`NaN`) en la aceleración de su controlador PID integral y generando saltos del robot.
