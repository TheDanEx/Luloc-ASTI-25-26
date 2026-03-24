# Component: line_follower_ctrl (Modes)

## Propósito Arquitectónico
Implementa la lógica de seguimiento de línea mediante un controlador PID desacoplado de la actuación de motores de bajo nivel. Utiliza la entrada del `line_sensor` para guiar la cinemática diferencial del robot.

## Entorno y Dependencias
- **Modos:** Se integra en `modes_execute` bajo `MODE_FOLLOW_LINE`.
- **Control:** Depende de `motor_velocity_ctrl` para el control de velocidad en lazo cerrado de cada rueda.

## Interfaces de E/S (Inputs/Outputs)
- **Inputs:** Posición de línea en mm.
- **Outputs:** Velocidades objetivo ($m/s$) para motores izquierdo y derecho.

## Flujo de Ejecución Lógico
1. **Percepción:** Obtiene posición de línea. Si se pierde la línea, predice la posición basada en el último signo.
2. **PID:** Calcula el error respecto al centro (0 mm) y extrae el factor de corrección.
3. **Cinemática:** Aplica `BaseSpeed + Correction` y `BaseSpeed - Correction`.
4. **Actuación:** Envía las velocidades resultantes a los controladores de velocidad de motor.
5. **Telemetría:** Envía el estado completo a la cola de telemetría desacoplada.

## Funciones Principales y Parámetros
- `follow_execute(...)`: Función principal del bucle de control (500Hz).
- Parámetros Kp, Ki, Kd configurables vía Kconfig o MQTT.

## Puntos Críticos y Depuración
- **Saturación:** La corrección PID se clampa para no exceder la velocidad máxima física.
- **Predicción de Salida:** Crucial para recuperar la línea en curvas cerradas.

## Ejemplo de Uso e Instanciación
Se activa automáticamente al cambiar el estado del robot a `MODE_FOLLOW_LINE`.
