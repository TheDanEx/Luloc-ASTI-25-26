# micro-ROS Manager (robot_node)

## Propósito Arquitectónico
Gestiona el ciclo de vida del nodo de micro-ROS (`robot_node`) en el **Core 0**. Este componente centraliza todas las suscripciones y publicaciones del robot, actuando como puente entre el middleware ROS 2 y la lógica interna a través de la memoria compartida.

## Entorno y Dependencias
- **micro-ROS for ESP-IDF:** Requiere el componente `micro_ros_bridge` y la librería estática `libmicroros.a`.
- **Shared Memory:** Depende de `shared_memory` para el intercambio de datos entre núcleos.

## Interfaces de E/S (Inputs/Outputs)
### Suscriptores
- `/robot/mode_cmd` (`std_msgs/msg/Int8`): Cambios de estado operativo.
- `/cmd_vel` (`geometry_msgs/msg/Twist`): Comandos de velocidad lineal y angular.

### Publicadores
- `/robot/line_sensors` (`std_msgs/msg/Float32MultiArray`): 10Hz. 8 Raw + 8 Normalized.
- `/robot/pid` (`std_msgs/msg/Float32MultiArray`): 10Hz. Error, Setpoint, Output, P, I, D.
- `/robot/calibration` (`std_msgs/msg/Float32MultiArray`): 1Hz. 8 Min + 8 Max.
- `/robot/odometry` (`std_msgs/msg/Float32MultiArray`): 1Hz. Pos X, Pos Y, Theta, Vel Lin, Vel Ang.
- `/robot/state` (`std_msgs/msg/Float32MultiArray`): 1Hz. Modo, Batería, Latencia, Jitter, Uptime.
- `/robot/logs` (`std_msgs/msg/String`): Mensajes de diagnóstico y eventos (formato ILP).

## Flujo de Ejecución Lógico
1. **Inicialización:** Crea una tarea dedicada en el Core 0.
2. **Sincronización:** Gestión de tiempo con el Agent.
3. **Timer Fast (10Hz):** Publica datos críticos para el control (Sensores y PID).
4. **Timer Slow (1Hz):** Publica datos de estado y diagnóstico menos frecuentes.
5. **Callbacks:** Inyecta comandos directamente en `shared_memory`.

## Funciones Principales y Parámetros
- `uros_manager_start()`: Crea la tarea de FreeRTOS y lanza el nodo.

## Puntos Críticos y Depuración
- **Downsampling:** El control corre a 100Hz, pero la telemetría se publica a 10Hz/1Hz para optimizar ancho de banda.
- **Asignación de Memoria:** Los buffers de los mensajes se alojan en el heap de FreeRTOS durante la inicialización del nodo.
