# Task Monitor Low-Power (task_monitor_lowpower_cpu1)

## Propósito Arquitectónico
Servicio mínimo de latido del sistema (Heartbeat pasivo) e impresor de Uptime. Sirve como prueba de vida inofensiva o "WatchDog a la escucha" para verificar que al menos el planificador (Scheduler) de FreeRTOS y el núcleo secundario no sufren un congelamiento de Kernel fatal, enviando un pulso de aviso ligero periódico.

## Entorno y Dependencias
- Tarea ultra-ligera de FreeRTOS situada en el **Core 1** con **Prioridad 1** (la prioridad más baja justo arriba de la tarea Idle).
- Emplea el driver `test_sensor` intrínseco de timers de alto rendimiento del chip y `mqtt_custom_client` para reporte.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Temporizador del sistema.
- **Software:** Disemina la vida del robot tanto en la terminal de depuración Serial (RX/TX UART) como en la nube a través del tópico corto `robot/uptime`.

## Flujo de Ejecución Lógico
Arranque aplazado (Delay preventivo de 10 segundos) evitando entorpecer la secuencia pesada inicial de boot. 
En ciclo perpetuo evalúa el temporizador de tiempo arriba nativo, construye la string (ej `1h 23m 45s`), lo avisa por logger INFO y, si hay portadora MQTT sana, lo publica asegurado (`QoS 1`), durmiendo de nuevo un generoso lapso de 5000 milisegundos ininterrumpidos.

## Funciones Principales y Parámetros
- `task_monitor_lowpower_cpu1_start(void)`: Función expuesta para instanciar la tarea, alojando apenas 2048 bytes de pila (Stack).

## Puntos Críticos y Depuración
- **Uptime Bloqueado / QoS:** Al publicar en MQTT con QoS=1 (`mqtt_custom_client_publish("robot/uptime", uptime_str, 0, 1, 0)`), la pequeña tarea garantiza su llegada. Si Mosquitto muere o la red falla misteriosamente, este QoS puede acular mensajes o rechazar la publicación (la lógica actualmente suprime warnings del logreo serial inteligentemente).
- **Indicador de Sobrecarga:** Debido a su Prioridad=1, si esta tarea en particular comienza a faltar a sus promesas de tiempo en Grafana o consola, es síntoma clarísimo e innegable de Starvation (Hambruna de recursos) indicando que otra tarea en el Core 1 está bloqueando la CPU permanentemente (`task_comms_cpu1` comúnmente).
