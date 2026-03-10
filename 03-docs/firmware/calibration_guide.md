# Guía de Calibración de PIDs en Vivo (Live Tuning)

Esta guía explica el proceso recomendado para sintonizar las ganancias de los lazos de control cerrados PID (Proporcional, Integral, Derivativo) utilizando las nuevas capacidades Asíncronas de la Máquina de Estados y el protocolo MQTT, sin necesidad de recompilar el firmware del ESP32.

## Arquitectura de Calibración

El robot diferencia entre sus Estados Operativos (`AUTONOMOUS`, etc.) y los **Modos de Sintonización**. Antes de inyectar variables de PID, el operador **debe** poner al robot en el modo de calibración correspondiente para asegurarse de que el RTOS active los sensores necesarios, ignore comandos externos contradictorios (como la IA de la cámara o Gamepads), y fije velocidades en lazo abierto constantes para observar respuestas limpias al escalón (Step Responses).

---

## 1. Calibración de Velocidad de Motores (`MODE_CALIBRATE_MOTORS = 5`)

**Objetivo:** Conseguir que el lazo de velocidad traduzca el comando "Gira a 1 metro/segundo" en un giro perfecto e instantáneo de 1 m/s en la rueda física, sorteando rozamientos mecánicos y caídas de voltaje de la batería, sin oscilar excesivamente.

### Preparación Física y del Sistema
1. **Física:** Eleve el robot en un banco de pruebas (ruedas en el aire sin tocar el suelo) o acomódelo en una pista recta e infinita.
2. Inicie el robot y confirme la conexión MQTT usando su dashboard (ej: MQTT Explorer o Grafana).
3. **Sweeper de Velocidad:** Previamente, en `menuconfig` (Bajo `[UJI] 1. Control: Motor Velocity -> Live Calibration Sweeper`), defina las dos velocidades que desea probar (ej. `0.5` m/s y `-0.5` m/s) y el tiempo que desea mantener cada paso (ej. `3000` ms). 
4. Pida a la API el cambio a modo calibración de motores:
   ```json
   // Topic: robot/api/set
   {"action": "set_mode", "mode_id": 5}
   ```
5. Verifique que la respuesta MQTT confirme la aceptación. En este estado, el robot ejecutará un bucle infinito ("Sweeping") escalando y estabilizando automáticamente entre las dos velocidades objetivo configurables, ideal para ver la Respuesta al Escalón en tiempo real.

### Proceso de Sintonización vía NVS / MQTT
Abra su Dashboard de inyección o terminal MQTT, y observe paralelamente el stream de RPMs/MTS en Grafana:

1. **Ajuste del Feed-Forward y Proporcional (Kp):**
   Mande un JSON completo apagando la memoria I y D. Vaya incrementando `Kp` hasta que la rueda se acerque agresivamente a la velocidad objetivo, pero comience a "tabletear" o vibrar.
   ```json
   // Topic: robot/config/pid_motors
   {"kp": 0.5, "ki": 0.0, "kd": 0.0}
   ```

2. **Ajuste Integral (Ki):**
   Notará que una `Kp` pura siempre deja un pequeño hueco (Steady State Error) porque al acercarse a 0.8m/s, el error baja, el PWM baja, y el rozamiento frena la rueda estabilizándola por debajo (ej. a 0.72 m/s). 
   Inyecte valores mínimos (muy sensibles) de `Ki` para acumular ese error en el tiempo y forzar a la rueda a llegar a 0.80 m/s exactos.
   ```json
   // Topic: robot/config/pid_motors
   {"kp": 0.45, "ki": 0.05} // Solo enviamos las variables modificadas
   ```

3. **Ajuste Derivativo (Kd) - Opcional:**
   Si al arrancar el motor se pasa frenéticamente de 0.8 a 1.2 m/s antes de estabilizarse (Overshoot), introduzca un leve amortiguador `Kd`.

*Nota: Cualquier JSON enviado a `robot/config/pid_motors` se guardará para siempre en la memoria Flash NVS del ESP32, sobreviviendo a apagados.*

---

## 2. Calibración del Sigue-Líneas (`MODE_CALIBRATE_LINE = 6`)

**Objetivo:** Conseguir que el robot sea capaz de mantenerse en el centro geométrico de la cinta eléctrica negra a alta velocidad sin temblores (Wobbling) bruscos ni salidas por la tangente en curvas escarpadas.

### Preparación Física y del Sistema
1. **Física:** Coloque el robot en el centro del circuito físico de ASTI sobre una recta que desemboque en una curva estándar. Evite probar los cruces ciegos en esta fase inicial.
2. Pida el cambio a modo de calibración de rastreador:
   ```json
   // Topic: robot/api/set
   {"action": "set_mode", "mode_id": 6}
   ```

En este modo `6`, el robot asume automáticamente control autónomo en movimiento continuo, usando estrictamente los datos del array de infrarrojos (o de la cámara si el hardware está ensamblado), ignorando semáforos o barreras externas.

### Proceso de Sintonización
El proceso requerirá perseguir al robot con la vista. (Asumiendo que has creado un Topic para el seguimiento de línea como `robot/config/pid_line`):

1. **Puesta a Cero e Impulso Base:**
   Anule las integrales y derivadas. Suba `Kp` gradualmente desde valores diminutos. Un Kp bajo hará que el robot tome las curvas muy "tarde" y se salga.
   ```json
   // Topic: robot/config/pid_line
   {"kp": 0.8, "ki": 0.0, "kd": 0.0}
   ```

2. **Doma de la Oscilación (Kd):**
   Cuando suba `Kp` lo suficiente como para no perderse en las curvas, notará que el robot parece estar ebrio al volver a la línea recta, dando bandazos a izquierda y derecha cada vez corrigiendo de más (Wobbling). 
   El parámetro rey en los Siguelíneas es el **Derivativo**. Inyecte mucha "Kd" para amortiguar los cambios bruscos de orientación.
   ```json
   // Topic: robot/config/pid_line
   {"kp": 1.2, "ki": 0.0, "kd": 0.5} 
   ```

3. **Corrección de Borde (Ki):**
   Solo si nota que el robot tiende a estar ligeramente ladeado crónicamente en pistas asimétricas, introduzca valores extremadamente bajos de `Ki` (ej. `0.001`). Normalmente un robot diferencial reacciona mal a integrales altas en guiñada (Yaw) provocando lo que se llama el "bucle de la muerte". 

4. **Calibración Predictiva de Cámara (Feed-Forward de Visión):**
   Si la etapa anterior es matemáticamente perfecta (Lazo reactivo), active el inyector del factor de curva predictivo.
   ```json
   {"ca_weight": 0.2}
   ```
   Aumente este factor hasta que note que el robot comienza a tumbar los motores asimétricamente milisegundos _antes_ de que el propio sensor de línea inferior pise el final de una curva matemática.

### Fin de Calibración
Una vez satisfecho con el rendimiento visual, cambie el modo de nuevo a Telemetría inactiva para detener las cadenas tractoras, o proceda al Modo Autónomo Completo (`mode_id: 1`).
```json
// Topic: robot/api/set
{"action": "set_mode", "mode_id": 4} // Modo seguro (TELEMETRY_STREAM)
```
