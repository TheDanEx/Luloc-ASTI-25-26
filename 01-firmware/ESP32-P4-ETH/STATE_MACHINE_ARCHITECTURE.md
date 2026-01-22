# MÁQUINA DE ESTADOS DEL SISTEMA - Arquitectura de Control

## DIAGRAMA DE ESTADOS

```
                    ┌─────────────────────────────────────────┐
                    │            INIT (startup)              │
                    │  - Esperando MQTT conectado           │
                    └──────────────┬──────────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │ MQTT conectado               │
                    │ timeout 2s                   │
                    ▼                              ▼
        ┌──────────────────────────┐   ┌───────────────────┐
        │  REMOTE_CONTROLLED       │   │   AUTONOMOUS      │
        │  Modo: MODE_REMOTE_DRIVE │   │   Modo: Auto      │
        │  - Espera comandos MQTT  │   │   - Sin MQTT req. │
        │  - Requiere MQTT siempre │   │   - Ejecuta paths │
        │  - Envía telemetría      │   │   - Sin telemetría│
        └──────────┬───────────────┘   └────────┬──────────┘
                   │                             │
                   │ MQTT OK                     │ MQTT OK
                   │ + Auto path                 │
                   └──────────────┬──────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    ▼
        ┌──────────────────────────────────────────┐
        │   TELEMETRY_ONLY                         │
        │   Modo: MODE_TELEMETRY_STREAM            │
        │   - Ejecuta control autónomo             │
        │   - Envía telemetría continuamente       │
        │   - Requiere MQTT para transmitir datos  │
        └──────────┬───────────────────────────────┘
                   │
                   │ MQTT PERDIDO
                   │ (CRÍTICO - PARADA SEGURA)
                   ▼
        ┌──────────────────────────────────────────┐
        │   MQTT_LOST_ERROR                        │
        │   ❌ MOTOR PARADO                        │
        │   ❌ SIN CONTROL                         │
        │   - Requiere intervención manual         │
        │   - Intenta recuperar si MQTT vuelve (>5s)
        └──────────┬───────────────────────────────┘
                   │
                   │ MQTT reconectado + 5s
                   ▼
        ┌──────────────────────────────────────────┐
        │   REMOTE_CONTROLLED (recovery)           │
        └──────────────────────────────────────────┘
```

---

## FLUJO DE COMUNICACIÓN INTER-CORE

### CPU 0 (Real-Time Control)

```
task_rtcontrol_cpu0
│
├─ state_machine_update()
│  ├─ Lee shared_memory.mqtt_connected (CPU1 lo comunica)
│  ├─ Gestiona transiciones de estado
│  └─ Toma decisiones según estado
│
├─ Si state == MQTT_LOST_ERROR:
│  │  STOP_MOTOR()
│  │  DISABLE_CONTROL()
│
├─ Si state == AUTONOMOUS:
│  │  EXECUTE_AUTONOMOUS_PATH()
│  │  NO_TELEMETRY_REQUIRED
│
├─ Si state == REMOTE_CONTROLLED:
│  │  WAIT_FOR_COMMANDS()
│  │  via shared_memory.last_command
│
└─ Si state == TELEMETRY_ONLY:
   │  EXECUTE_CONTROL()
   │  WRITE_SENSOR_DATA()
   │  via shared_memory.sensors
```

### CPU 1 (Communications)

```
task_comms_cpu1
│
├─ Mantiene conexión MQTT
│
├─ En mqtt_event_handler():
│  ├─ MQTT_EVENT_CONNECTED:
│  │  └─ shared_memory_set_mqtt_connected(true)
│  │
│  └─ MQTT_EVENT_DISCONNECTED:
│     └─ shared_memory_set_mqtt_connected(false)
│
└─ Cada 5s: task_monitor_lowpower_cpu1
   └─ Verifica estado y publica telemetría
```

---

## TABLA DE TRANSICIONES

| Estado Actual     | MQTT Status        | Acción                   | Nuevo Estado      |
| ----------------- | ------------------ | ------------------------ | ----------------- |
| INIT              | ✓ Conectado        | Configura remote control | REMOTE_CONTROLLED |
| INIT              | ✗ No conectado     | Espera                   | INIT              |
| REMOTE_CONTROLLED | ✗ Perdido          | PARADA SEGURA            | MQTT_LOST_ERROR   |
| AUTONOMOUS        | ✓ Conectado        | Habilita telemetría      | TELEMETRY_ONLY    |
| TELEMETRY_ONLY    | ✗ Perdido          | PARADA SEGURA            | MQTT_LOST_ERROR   |
| MQTT_LOST_ERROR   | ✓ Recuperado (>5s) | Reintenta                | REMOTE_CONTROLLED |

---

## DECISIONES DE CONTROL POR ESTADO

### STATE_AUTONOMOUS

- ✅ Ejecuta paths pre-programados
- ✅ Control sin necesidad de conexión
- ✅ NO envía telemetría
- ⚠️ No puede recibir comandos remotos

### STATE_REMOTE_CONTROLLED

- ✅ Espera comandos via MQTT
- ✅ Envía telemetría continuamente
- ❌ **REQUIERE MQTT conectado**
- ❌ Si MQTT se pierde → ERROR

### STATE_TELEMETRY_ONLY

- ✅ Ejecuta autonomía + telemetría
- ✅ Mejor que REMOTE_CONTROLLED
- ❌ **REQUIERE MQTT para telemetría**
- ❌ Si MQTT se pierde → ERROR

### STATE_MQTT_LOST_ERROR

- ❌ Motor detenido
- ❌ Sin control remoto
- ❌ Sin telemetría
- ⏳ Espera recuperación de MQTT (>5s)
- 🔧 Requiere intervención manual si MQTT no se recupera

---

## INTEGRACIÓN: COMPONENTES INVOLUCRADOS

```
┌─────────────────────────────────────────────────────────────┐
│                        app_main.c                           │
│  - Inicializa todas las dependencias en orden correcto      │
└─────────────────────────────────────────────────────────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐        ┌───────────┐
    │shared_  │         │state_   │        │mqtt_      │
    │memory   │◄────────│machine  │───────►│watchdog   │
    └─────────┘         └─────────┘        └───────────┘
         ▲                   │                    │
         │                   │                    │
         └───────────────────┼────────────────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
    ┌────────────┐   ┌──────────────┐   ┌─────────────┐
    │task_rtctrl │   │task_comms_   │   │task_monitor │
    │_cpu0       │   │cpu1          │   │_lowpower    │
    │            │   │              │   │_cpu1        │
    │CPU 0:      │   │CPU 1:        │   │CPU 1:       │
    │- Control   │   │- MQTT client │   │- Telemetría│
    │- Sensores  │   │- Watchdog    │   │- Monitoreo │
    │- Est.Máqn. │   │- COM buffer  │   │- Log serial│
    └────────────┘   └──────────────┘   └─────────────┘
```

---

## EJEMPLO: CICLO DE OPERACIÓN COMPLETO

### T=0s: Boot

1. app_main() inicia
2. Ethernet + MQTT start
3. task_comms_cpu1_start()
4. Espera 2s por MQTT

### T=2.5s: MQTT Conectado

1. task_comms_cpu1_is_ready() = true
2. task_rtcontrol_cpu0_start()
3. shared_memory_set_mqtt_connected(true)
4. state_machine_update() → INIT → REMOTE_CONTROLLED

### T=5s: Operación Normal

1. task_rtcontrol_cpu0: Espera comandos MQTT
2. task_comms_cpu1: Lee de broker
3. task_monitor_lowpower_cpu1: Publica uptime

### T=30s: MQTT Se Desconecta (Red perdida)

1. mqtt_event_handler() → MQTT_EVENT_DISCONNECTED
2. shared_memory_set_mqtt_connected(false)
3. state_machine_update() detecta cambio
4. Transición: REMOTE_CONTROLLED → MQTT_LOST_ERROR
5. ❌ Motor PARADO inmediatamente
6. 🔴 Sistema en ERROR esperando intervención

### T=35s: MQTT Recupera

1. mqtt_event_handler() → MQTT_EVENT_CONNECTED
2. shared_memory_set_mqtt_connected(true)
3. shared_memory.mqtt_connected = true
4. state_machine_update() detect+5s timeout
5. Transición: MQTT_LOST_ERROR → REMOTE_CONTROLLED
6. ✅ Sistema vuelve a operar

---

## CONFIGURACIÓN RECOMENDADA

### Para Telerobotics Puro (Control remoto):

```
MODE → MODE_REMOTE_DRIVE
- Requiere MQTT siempre
- Si falla → PARADA INMEDIATA
```

### Para Robot Autónomo + Telemetría:

```
MODE → MODE_TELEMETRY_STREAM
- Ejecuta autonomía
- Envía datos en tiempo real
- Si falla MQTT → PARADA (no hay feedback)
```

### Para Robot Autónomo Puro:

```
MODE → MODE_AUTONOMOUS_PATH
- Ejecuta sin MQTT
- Más robusto ante fallos de red
- Sin monitoreo remoto
```

---

## MONITOREO Y DEBUG

Cada cambio de estado loguea:

```
[state_machine] Mode transition: 0 → 1
[state_machine] [EVENT] MQTT CONNECTED - state: REMOTE_CONTROLLED, mode: 1
[state_machine] [EVENT] MQTT DISCONNECTED - state: REMOTE_CONTROLLED, mode: 1
[state_machine] CRITICAL: MQTT lost during REMOTE_CONTROLLED mode - STOPPING
```

Ver en monitor serial:

```
idf.py -p COM_PORT monitor | grep -E "state_machine|MQTT|ERROR"
```
