# Robot ESP32 - Documentación de API MQTT Genérica

Este documento consolida la totalidad de tópicos y payloads (Mensajes) que maneja el cerebro robótico ESP32 a través del protocolo MQTT, siguiendo el **Estándar de Comunicación MQTT**.

## 1. Eventos Asíncronos Críticos (Outputs)
*Gestionado por: `state_machine` y componentes de control.*

**Tópico:** `robot/events`
Format: JSON.

| Evento | Payload |
| :--- | :--- |
| Cambio de Modo | `{"event":"MODE_CHANGE","mode":2,"mode_str":"AUTONOMOUS"}` |
| Error de Modo | `{"error":"MODE_CHANGE_REJECTED"}` |

---

## 2. Telemetría Line Protocol (Push Batch)
*Gestionado por: `telemetry_manager`.*

**Tópicos:** `robot/telemetry/<measurement>`
Formato: ILP con Timestamps de 19 dígitos (Nanosegundos).

| Tópico | Measurement | Campos Comunes |
| :--- | :--- | :--- |
| `robot/telemetry/odometry` | `odometry` | `velIZ`, `posIZ`, `velDR`, `posDR` |
| `robot/telemetry/system` | `system` | `uptime_sec`, `uptime_ms`, `curvatura_ff` |

---

## 3. Logs y Debug (Outputs)
*Gestionado por: `mqtt_custom_client`.*

- **Logs:** `robot/logs/<level>` (error, warn, info, debug).
- **Debug Sandbox:** `robot/debug` (Para volcado de datos crudos).

---

## 4. API Unificada (Request / Response)
*Gestionado por: `mqtt_api_responder`.*

**Tópico Request:** `robot/api/request`
**Tópico Response:** `robot/api/response`

Toda petición debe incluir el campo `"op"` (`get` o `set`).

### OPERACIÓN: GET (Consulta de Recursos)
Envío: `{"op": "get", "resource": "<nombre>"}`

1. **Batería:** `{"op": "get", "resource": "battery"}` -> `{"op": "resp", "resource": "battery", "battery_mv": 16400, ...}`
2. **Encoder:** `{"op": "get", "resource": "encoder"}` -> `{"op": "resp", "resource": "encoder", "speed_l_ms": 1.2, ...}`
3. **Uptime:** `{"op": "get", "resource": "uptime"}` -> `{"op": "resp", "resource": "uptime", "uptime_sec": 3491, ...}`
4. **All:** `{"op": "get", "resource": "all"}` -> Retorna todos los anteriores en un solo objeto.

### OPERACIÓN: SET (Acciones y Comandos)
Envío: `{"op": "set", "action": "<nombre>", ...}`

1. **Reproducir Sonido:** `{"op": "set", "action": "play_sound", "sound_id": 1, "volume": 80}`
2. **Cambiar de Modo:** `{"op": "set", "action": "set_mode", "mode_id": 1, "force": false}`
3. **Mascara Calibración:** `{"op": "set", "action": "set_cal_mask", "mask": 3}`

---

## 5. Configuración (NVS)
*Gestionado por: `pid_tuner` y `curvature_feedforward`.*

**Tópicos:** `robot/config/<subsystem>`

- **Motores:** `robot/config/motors` -> `{"kp": 0.8, "ki": 0.1, "kd": 0.05}`
- **Curvatura:** `robot/config/curvature` -> `0.05` (Float o Binario).
