# Guía de Calibración de PIDs en Vivo (Live Tuning)

Esta guía explica el proceso recomendado para sintonizar las ganancias de los lazos de control cerrados PID (Proporcional, Integral, Derivativo) utilizando las nuevas capacidades Asíncronas de la Máquina de Estados y el protocolo MQTT, sin necesidad de recompilar el firmware del ESP32.

## Arquitectura de Calibración

El robot diferencia entre sus Estados Operativos (`AUTONOMOUS`, etc.) y los **Modos de Sintonización**. Antes de inyectar variables de PID, el operador **debe** poner al robot en el modo de calibración correspondiente para asegurarse de que el RTOS active los sensores necesarios, ignore comandos externos contradictorios (como la IA de la cámara o Gamepads), y fije velocidades en lazo abierto constantes para observar respuestas limpias al escalón (Step Responses).

---

## 1. Calibración de Velocidad de Motores (`MODE_CALIBRATE_MOTORS = 5`)

**Objetivo:** Conseguir que el lazo de velocidad traduzca el comando "Gira a 1 m/s" en un movimiento físico real, compensando fricción y batería.

### Preparación del Sistema
1. **Selección de Motor:** Antes de empezar, elija qué motor desea mover usando el comando `set_cal_mask`:
   ```json
   // Topic: robot/api/request
   {"op": "set", "action": "set_cal_mask", "mask": 1} // 1=Izquierdo, 2=Derecho, 3=Ambos
   ```
2. **Entrada en Modo:** Cambie al modo de calibración:
   ```json
   {"op": "set", "action": "set_mode", "mode_id": 5}
   ```

### Ajuste dinámico del Barrido (Sweeper)
No es necesario recompilar para cambiar la velocidad del test. Puede configurar el barrido o fijar una velocidad manual:
```json
// Topic: robot/config/calibration
{"speed1": 0.5, "speed2": -0.5, "interval_ms": 2000, "manual_mode": false}
```
*Si activa `manual_mode: true`, el robot mantendrá la velocidad `manual_speed` de forma constante.*

### Sintonización Independiente vía MQTT
Envíe las ganancias al tópico `robot/config/motors`. Puede especificar el motor para un ajuste fino independiente:
```json
// Topic: robot/config/motors
{"motor": "left", "kp": 1.2, "ki": 0.1, "kd": 0.05}
```

### Análisis de Telemetría Avanzada
Observe el tópico `robot/telemetry/calibration` (Grafana). Ahora el sistema desglosa el origen de cada milivoltio aplicado al motor:
*   **`v_ff`**: Voltaje base (Feed-Forward + Deadband).
*   **`v_p`**: Aporte proporcional (Corrección de error).
*   **`v_i`**: Aporte integral (Eliminación de error estático).
*   **`v_d`**: Aporte derivativo (Amortiguación).
*   **`v_final`**: Suma total aplicada al driver.

---

## 2. Calibración del Sigue-Líneas (`MODE_FOLLOW_LINE = 2`)

**Objetivo:** Mantener el centro de la línea negra sin oscilaciones (Wobbling) y con suavidad en curvas.

### Preparación
1. Coloque el robot en pista y active el modo:
   ```json
   {"op": "set", "action": "set_mode", "mode_id": 2}
   ```

### Configuración Dinámica
A diferencia de los motores, el siguelíneas tiene su propio tópico de configuración JSON:
```json
// Topic: robot/config/follow_line
{
  "kp": 1.5, 
  "kd": 0.5, 
  "max_speed": 0.8,
  "ff_weight": 0.4
}
```

### Parámetros Clave
*   **`kp`**: Agresividad al corregir el error de posición.
*   **`kd`**: Amortiguación. Vital para evitar el "culebreo" en rectas tras salir de una curva.
*   **`ff_weight`**: (0.0 a 1.0). Determina cuánto caso le hace el robot a la predicción de curva de la Raspberry Pi. Si el robot frena demasiado en curvas, baje este valor.

### Configuración por Defecto (Kconfig)
Los valores iniciales se pueden definir en `idf.py menuconfig` bajo el menú `Follow Line Configuration`, lo que permite que el robot arranque con una base estable.

---

## 3. Ejemplo Paso a Paso: Calibración Rápida

Para calibrar el **Motor Izquierdo** desde cero:

1.  **Activar Trazas:** Ponga al robot en el modo adecuado:
    `Topic: robot/api/request` -> `{"op": "set", "action": "set_mode", "mode_id": 5}`
2.  **Aislar el Motor:** Active solo el motor izquierdo:
    `Topic: robot/api/request` -> `{"op": "set", "action": "set_cal_mask", "mask": 1}`
3.  **Lanzar Estímulo:** Configure un barrido de 0.5 m/s cada 2 segundos:
    `Topic: robot/config/calibration` -> `{"speed1": 0.5, "speed2": 0.0, "interval_ms": 2000}`
4.  **Ajustar Ganancias:** Inyecte valores hasta que la curva `actual_l` siga a `target_l`:
    `Topic: robot/config/motors` -> `{"motor": "left", "kp": 1.5, "ki": 0.2}`
5.  **Validar Diagnóstico:** En Grafana, asegúrese de que `v_i_l` no sature y que `v_p_l` sea el que mande en los transitorios.

> [!IMPORTANT]
> **Aviso de Persistencia**
> *   **Motores:** Los valores enviados a `robot/config/motors` **SÍ** se guardan en la memoria Flash (NVS) y sobreviven al reinicio.
> *   **Siguelíneas y Barrido:** Los valores de `follow_line` y `calibration` son **VOLÁTILES**. Se borrarán al apagar el robot.
> *   **Recomendación:** Una vez hallados los valores óptimos, **trasládelos a `menuconfig`** (`idf.py menuconfig`) para que formen parte del firmware oficial y el robot sea determinista desde el segundo 0.

---

## 4. Resumen de Tópicos
| Función | Tópico MQTT | Payload Ejemplo |
| :--- | :--- | :--- |
| **Control Motores** | `robot/config/motors` | `{"motor":"left", "kp":1.0}` |
| **Test de Barrido** | `robot/config/calibration` | `{"speed1":0.5, "manual_mode":false}`|
| **Sigue-Líneas** | `robot/config/follow_line` | `{"kp":1.2, "ff_weight":0.5}` |
| **Comandos API** | `robot/api/request` | `{"op":"set", "action":"set_mode", "mode_id":2}` |
