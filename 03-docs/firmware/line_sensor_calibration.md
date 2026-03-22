# Guía de Calibración del Sensor de Línea

Esta guía detalla el proceso para calibrar el array de sensores infrarrojos (reflectancia) y sintonizar el algoritmo de seguimiento de línea (PID).

## 1. Calibración de Reflectancia (`MODE_CALIBRATE_MOTORS` + Sensor Check)

Para que el robot distinga correctamente entre el suelo (blanco) y la línea (negra), cada sensor debe mapear su rango dinámico (Min/Max).

### Procedimiento de Calibración
1. **Poner el robot en pista**: Asegúrese de que el array de sensores esté sobre una zona con línea negra y fondo blanco.
2. **Activar Movimiento**: Use el modo de calibración de motores para mover el robot lateralmente sobre la línea, o muévalo a mano si prefiere:
   ```json
   // Topic: robot/api/request
   {"op": "set", "action": "set_mode", "mode_id": 5}
   ```
3. **Monitorear Rangos**: Observe el tópico `robot/telemetry/line`. Verá que los campos `minX` y `maxX` se actualizan a medida que los sensores ven blanco y negro.
4. **Validación**: Una vez que todos los sensores hayan pasado por ambos colores, el flag `is_calibrated` en la telemetría se pondrá en `true`.

---

## 2. Sintonización del PID de Línea (`MODE_FOLLOW_LINE = 2`)

Una vez calibrados los sensores, puede ajustar el comportamiento del robot al seguir la línea.

### Tópico de Configuración
```json
// Topic: robot/config/follow_line
{
  "kp": 1.5, 
  "ki": 0.01,
  "kd": 0.5, 
  "max_speed": 0.8,
  "ff_weight": 0.3
}
```

### Telemetría de Diagnóstico
Suscríbase a `robot/telemetry/line` para ver el estado en tiempo real:
- **`err`**: Desviación del centro de la línea (centros de masa). 0.0 es centro perfecto.
- **`target_l` / **`target_r`**: Velocidad objetivo (m/s) calculada por el PID para cada motor.
- **`s0` a `s7`**: Valores normalizados (0.0 a 1.0) de cada sensor. Útil para detectar sensores sucios o mal alineados.
- **`kp`, `ki`, `kd`**: Confirma que el robot ha recibido tus cambios de PID.

### Consejos de Sintonización
*   **Oscilación en rectas**: Si el robot "serpentea" mucho en las rectas, suba el valor de `kd`.
*   **Pérdida en curvas**: Si el robot se sale por el exterior de la curva, suba `kp`.
*   **Curvatura RPi**: El valor `ff_weight` controla la importancia de los datos de visión de la Raspberry Pi. 0.0 ignora la cámara, 1.0 le da prioridad máxima.

---

## 3. Resumen de Herramientas
- **MQTT Explorer**: Para ver los valores crudos y enviar comandos JSON.
- **Grafana**: Para visualizar las 8 señales de los sensores simultáneamente y ver cómo evoluciona el `error` frente a la respuesta de los motores.
