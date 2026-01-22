# MQTT COMMANDS & EVENTS - Robot Communication Protocol

## Overview

El robot ahora escucha en `robot/cmd` para cambios de modo y publica eventos de estado en `robot/events`.

---

## MQTT TOPICS

### Input: `robot/cmd`

**Recibe comandos para cambiar el modo de operación**

- **QoS**: 1 (At least once)
- **Format**: Plain text UTF-8
- **Subscribers**: ESP32 (task_comms_cpu1)

### Output: `robot/events`

**Publica eventos de cambio de estado**

- **QoS**: 1 (At least once)
- **Format**: Plain text UTF-8
- **Publishers**: ESP32 (state_machine)

---

## MODOS DISPONIBLES

### 1. MODE_AUTONOMOUS_PATH

```
Comando MQTT:
  Topic: robot/cmd
  Payload: MODE_AUTONOMOUS

Comportamiento:
  ✓ Ejecuta path pre-programado
  ✓ NO requiere MQTT para operar
  ✓ NO envía telemetría
  ✓ NO puede recibir comandos remotos

Útil para:
  - Misiones autónomas
  - Entornos sin conexión
  - Bajo consumo de ancho de banda
```

### 2. MODE_REMOTE_DRIVE

```
Comando MQTT:
  Topic: robot/cmd
  Payload: MODE_REMOTE_DRIVE

Comportamiento:
  ✓ Espera comandos via MQTT
  ✓ REQUIERE MQTT conectado siempre
  ✓ Envía telemetría continuamente
  ❌ Si MQTT cae → MOTOR PARADO inmediatamente

Útil para:
  - Control remoto en tiempo real
  - Joystick / teleoperación
  - Operación asistida
```

### 3. MODE_TELEMETRY_STREAM

```
Comando MQTT:
  Topic: robot/cmd
  Payload: MODE_TELEMETRY_STREAM

Comportamiento:
  ✓ Ejecuta autonomía + telemetría
  ✓ REQUIERE MQTT para publicar datos
  ❌ Si MQTT cae → MOTOR PARADO inmediatamente

Útil para:
  - Autonomía con monitoreo remoto
  - Misiones supervisadas
  - Mejor que REMOTE_DRIVE para autonomía real
```

---

## EVENTOS DE ESTADO

El robot publica automáticamente cuando:

```
STATE_REMOTE_CONTROLLED
  → Modo de control remoto activo
  → Esperando comandos

STATE_AUTONOMOUS
  → Ejecutando autonomía
  → Sin telemetría requerida

STATE_TELEMETRY_ONLY
  → Autonomía + transmitiendo datos

STATE_MQTT_LOST_ERROR - MQTT_DISCONNECTED
  → CRÍTICO: MQTT se perdió durante REMOTE_CONTROLLED
  → Motor detenido
  → Esperando reconexión de MQTT (>5s para recuperar)

STATE_MQTT_LOST_ERROR - TELEMETRY_FAILED
  → CRÍTICO: MQTT se perdió durante TELEMETRY_ONLY
  → Motor detenido

STATE_REMOTE_CONTROLLED - RECOVERED
  → Sistema se recuperó después de error MQTT
  → Volviendo a modo control remoto
```

---

## EJEMPLOS DE USO CON MOSQUITTO CLI

### Cambiar a modo autónomo:

```bash
mosquitto_pub -h 192.168.42.15 -p 51111 -t robot/cmd -m "MODE_AUTONOMOUS"
```

### Cambiar a control remoto:

```bash
mosquitto_pub -h 192.168.42.15 -p 51111 -t robot/cmd -m "MODE_REMOTE_DRIVE"
```

### Cambiar a telemetría con autonomía:

```bash
mosquitto_pub -h 192.168.42.15 -p 51111 -t robot/cmd -m "MODE_TELEMETRY_STREAM"
```

### Escuchar eventos:

```bash
mosquitto_sub -h 192.168.42.15 -p 51111 -t robot/events
```

### Simulación completa:

```bash
# Terminal 1: Monitor de eventos
mosquitto_sub -h 192.168.42.15 -p 51111 -t "robot/#"

# Terminal 2: Enviar comandos
sleep 5 && mosquitto_pub -h 192.168.42.15 -p 51111 -t robot/cmd -m "MODE_REMOTE_DRIVE"
sleep 10 && mosquitto_pub -h 192.168.42.15 -p 51111 -t robot/cmd -m "MODE_AUTONOMOUS"
sleep 5 && mosquitto_pub -h 192.168.42.15 -p 51111 -t robot/cmd -m "MODE_TELEMETRY_STREAM"
```

---

## ARQUITECTURA DE PROCESAMIENTO

```
┌─────────────────────────────────────────────────────┐
│              MQTT Broker (Remote)                   │
│  Topics: robot/cmd, robot/events, robot/uptime      │
└──────────────────────┬──────────────────────────────┘
                       │ (Network)
                       │
┌──────────────────────┴──────────────────────────────┐
│             ESP32-P4 Robot (Local)                  │
├──────────────────────────────────────────────────────┤
│                                                      │
│  CPU 1: task_comms_cpu1                            │
│  ├─ Recibe: robot/cmd (via mqtt_cmd_callback)      │
│  ├─ Procesa: Comando → state_machine_request_mode()│
│  └─ Publica: robot/events                          │
│                                                      │
│  CPU 0: task_rtcontrol_cpu0                        │
│  ├─ Ejecuta: state_machine_update()                │
│  ├─ Lee: shared_memory.mqtt_connected              │
│  ├─ Toma decisión: ¿Motor? ¿Control? ¿Parada?     │
│  └─ Resultado: Control actual del motor             │
│                                                      │
│  shared_memory                                      │
│  └─ mqtt_connected (CPU1→CPU0)                     │
│                                                      │
│  state_machine (CPU0)                              │
│  ├─ INIT → REMOTE_CONTROLLED → ...                │
│  └─ Publica eventos al cambiar                     │
│                                                      │
└──────────────────────────────────────────────────────┘
```

---

## FLUJO DE EJECUCIÓN COMPLETO

### Startup:

```
1. app_main() inicia Ethernet + MQTT
2. task_comms_cpu1 inicia
3. mqtt_custom_client_init() exitoso
4. Registra callback mqtt_cmd_callback para robot/cmd
5. Se suscribe a robot/cmd
6. Publica: "ESP32 MQTT connection started" → robot/events
```

### Cambio de modo (usuario envía comando):

```
1. Usuario: mosquitto_pub ... "MODE_REMOTE_DRIVE"
2. MQTT broker recibe en robot/cmd
3. ESP32 recibe mensaje
4. mqtt_cmd_callback() invocado
5. Parsea comando → MODE_REMOTE_DRIVE
6. Llama: state_machine_request_mode(MODE_REMOTE_DRIVE)
7. state_machine valida cambio (¿MQTT conectado?)
8. Si OK: cambia modo
9. publish_state_event("MODE_REMOTE_DRIVE")
10. Robot publica en robot/events
11. Usuario recibe confirmación
```

### Pérdida de MQTT (en REMOTE_CONTROLLED):

```
1. MQTT broker desconecta (network down, broker crash, etc.)
2. mqtt_event_handler() recibe MQTT_EVENT_DISCONNECTED
3. shared_memory_set_mqtt_connected(false)
4. task_rtcontrol_cpu0: state_machine_update()
5. Detecta: REMOTE_CONTROLLED + mqtt_connected=false
6. Transición: MQTT_LOST_ERROR
7. publish_state_event("STATE_MQTT_LOST_ERROR")
8. Motor PARADO inmediatamente
9. CPU0 no ejecuta control
10. Sistema espera (>5s) reconexión de MQTT
11. Cuando MQTT vuelve: intenta recuperar
```

---

## SEGURIDAD & ROBUSTEZ

✅ **Implementado:**

- Thread-safe shared memory con mutexes
- Validación de comandos antes de ejecutar
- PARADA SEGURA si falla MQTT en modos críticos
- Intentos de recuperación automática
- Logging detallado de todos los eventos

⚠️ **Consideraciones:**

- Los mensajes MQTT pueden llegar desordenados (usar timestamps)
- Conexión MQTT no garantiza baja latencia (puede ser 100-500ms)
- Para control real-time crítico: usar protocolo binario en lugar de MQTT
- Los comandos no tienen authentication (usar firewall/VPN)

---

## MONITOREO EN TIEMPO REAL

Ver solo cambios de estado:

```bash
mosquitto_sub -h 192.168.42.15 -p 51111 -t robot/events | grep -E "STATE_|ERROR"
```

Grabar todos los eventos:

```bash
mosquitto_sub -h 192.168.42.15 -p 51111 -t "robot/#" | tee robot_events_$(date +%s).log
```

Estadísticas en tiempo real (Python):

```python
import paho.mqtt.client as mqtt
import json
from datetime import datetime

def on_message(client, userdata, msg):
    print(f"[{datetime.now().isoformat()}] {msg.topic}: {msg.payload.decode()}")

client = mqtt.Client()
client.on_message = on_message
client.connect("192.168.42.15", 51111, 60)
client.subscribe("robot/#")
client.loop_forever()
```

---

## INTEGRACIÓN CON ROS / ROS2

Para integrar con ROS:

```bash
# ROS Bridge MQTT
sudo apt install ros-foxy-mqtt-bridge

# Configuración: robot_cmd_bridge.yaml
connections:
  - name: "mode_command"
    ros_topic: "/robot/cmd"
    mqtt_topic: "robot/cmd"
    ros_msg_type: "std_msgs/String"
    direction: "ros_to_mqtt"

  - name: "state_events"
    ros_topic: "/robot/state"
    mqtt_topic: "robot/events"
    ros_msg_type: "std_msgs/String"
    direction: "mqtt_to_ros"
```

---

## FUTURAS MEJORAS

- [ ] Agregar autenticación MQTT (usuario/contraseña o certificados)
- [ ] Implementar Keep-Alive personalizado
- [ ] Agregar compresión en comandos largos
- [ ] Timeout configurable para recuperación MQTT
- [ ] Cola de comandos si MQTT desconectado temporalmente
- [ ] Historial de eventos (ringbuffer local)
- [ ] Telemetría condicional (solo si cambia > threshold)
