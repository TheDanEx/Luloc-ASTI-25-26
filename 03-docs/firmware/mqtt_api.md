# Robot ESP32 - Documentación de API MQTT Genérica

Este documento consolida la totalidad de tópicos y payloads (Mensajes) que maneja el cerebro robótico ESP32 a través del protocolo MQTT. El sistema utiliza MQTT tanto de forma Asíncrona (Telemetría y Alertas) como de forma Síncrona (Peticiones JSON al vuelo y Settings NVS).

## 1. Comandos de Reducción Simples (Inputs Asíncronos)
*Gestionado por: `task_comms_cpu1` (RX).*

*(Deprecado en favor de la API Síncrona REST-Like JSON). El robot ya no procesa strings puros en `robot/cmd` para evitar fallos de parseo. Acuda a la sección final de este documento.*

---

## 2. Eventos Asíncronos Criticos (Outputs Asíncronos)
*Gestionado por: Componentes de Control de Lazo y `state_machine` (TX).*

Son "Pushes" instantáneos. No se suben a Influx, están diseñados para ser leídos por un Dashboard de operarios porque requieren acción inmediata.
**Tópico Base:** `robot/events`

| Condición de Disparo | Payload Generado |
| :--- | :--- |
| Cambio de Modo Exitoso | `{"event":"MODE_CHANGE","mode":2,"mode_str":"AUTONOMOUS"}` |
| Interlock Mecánico / Rechazo | `{"error":"MODE_CHANGE_REJECTED"}` |
| Pérdida de Enlace Crítico (Futuro)| `{"error":"Gamepad_Disconnect", "action":"Emergency_Brake"}` |

---

## 3. Streaming de Telemetría (Push Batch a InfluxDB)
*Gestionado por: `telemetry_manager` (TX).*

Son tópicos diseñados exclusivamente para enviar el *Line Protocol (ILP)* de InfluxDB. Se emiten cada pocos segundos (1000ms a 5000ms según Kconfig) concatenando las múltiples lecturas físicas para ahorrar latencia WiFi.

| Tópico de Salida | Propósito | Frecuencia Común |
| :--- | :--- | :--- |
| `robot/odometry` | Vierte la velocidad en m/s (velIZ, velDE) y posición en m (posIZ, posDE) del robot. | 5Hz Batched / 1 vez seg. |
| `robot/telemetry/power`| Vierte voltios `battery_mv` y miliamperios `motor_current` recolectados del INA219. | 5Hz Batched / 1 vez seg. |
| `robot/telemetry` | Varios: Curvatura matemática de Visión Arti., Uptime ticks, Profiling de Memoria Libre. | Variable (Lenta). |

---

## 4. Configuración Segura de Paramétros Lazo Cerrado (Input Síncrono-NVS)
*Gestionado por: `pid_tuner` (RX).*

Tópico exclusivo para inyectar sobre la marcha calibraciones matemáticas. Las recepciones sobrescriben el cerebro activo (`shared_memory`) y se "flashean" instantáneamente a la memoria ROM del ESP32 para sobrevivir reinicios.
**Tópico Base:** `robot/config/pid_motors`

| Payload JSON (Parcial o Total) | Descripción |
| :--- | :--- |
| `{"kp": 0.8, "ki": 0.2, "kd": 0.05}` | Resetea la integral e inyecta estas constantes instantáneamente a ambas ruedas motrices. |
| `{"kp": 0.9}` | Inyección parcial autorizada. Solo modifica la Kp y lee y re-flashea los valores residentes anteriores de Ki y Kd para no destruir datos por accidente. |

---

## 5. API Síncrona REST-Like (Pregunta / Respuesta)
*Gestionado por: `mqtt_api_responder` (RX/TX).*

Sirve para que sistemas externos, scripts en Python de supervisión, o ingenieros consulten el estado actual del robot sin esperar al siguiente tick de Telemetría de Influx. 
**Tópico Endpoint:** `robot/api/get` (Para preguntar)
**Tópico Output:** `robot/api/response` (Robot responde aquí el JSON resultante 3 milisegundos después).

Para interactuar con la API, escriber por MQTT al tópico **GET** un JSON con esta firma exacta: `{"resource": "<nombre_recurso>"}`

### Recursos Soportados:
1. Petición Batería: 
   - Envío: `{"resource": "battery"}`
   - Respuesta: `{"resource": "battery", "battery_mv": 16400, "robot_current_ma": 420.5}`
2. Petición Cinemática: 
   - Envío: `{"resource": "encoder"}`
   - Respuesta: `{"resource": "encoder", "speed_l_ms": 1.2, "speed_r_ms": 1.15, "ticks_l": 454322, "ticks_r": 454100}`
3. Petición Tiempos: 
   - Envío: `{"resource": "uptime"}`
   - Respuesta: `{"resource": "uptime", "uptime_sec": 3491, "uptime_us": 3491500000}`
4. Petición Total: 
   - Envío: `{"resource": "all"}`
   - Respuesta: `{"resource": "all", "battery_mv": 16400, "robot_current_ma": 420.5, "speed_l_ms": 1.2, "speed_r_ms": 1.15, "ticks_l": 454322, "ticks_r": 454100, "uptime_sec": 3491, "uptime_us": 3491500000}`
   
   *(Si un `resource` no existe en el código C del firmware, el robot retornará amistosamente un JSON informando `{"resource": "foo", "error": "Unknown Resource"}`).*

---

## 6. API Síncrona REST-Like (Comandos de Acción POST/SET)
*Gestionado por: `mqtt_api_responder` (RX/TX).*

Sustituye a los antiguos comandos de String permitiendo inyección de parámetros estructurados. Tras procesar la acción, el robot siempre responde indicando éxito o fracaso.
**Tópico Endpoint:** `robot/api/set` (Para mandar ejecutar acciones)
**Tópico Output:** `robot/api/response` (Robot responde aquí el JSON resultante confirmando la acción).

Para interactuar con la API, escriber por MQTT al tópico **SET** un JSON con esta firma exacta: `{"action": "<nombre_accion>", "param1": X, "param2": Y}`

### Acciones Soportadas:
1. Reproducir Sonido (Audio Player): 
   - Envío: `{"action": "play_sound", "sound_id": 1, "volume": 80}` (El campo `volume` es opcional).
   - Sonidos Enum disponibles:`0 = BATTERY_LOW`, `1 = STARTUP`
   - Respuesta Éxito: `{"action": "play_sound", "status": "success", "message": "Sound queued"}`
   - Respuesta Fallo: `{"action": "play_sound", "status": "error", "message": "Invalid sound_id"}`

2. Cambiar de Modo (State Machine):
   - Envío: `{"action": "set_mode", "mode_id": 1, "force": false}` (El campo `force` es opcional, false por defecto).
   - Modos Enum disponibles: `1 = PATH`, `2 = OBSTACLE`, `3 = REMOTE`, `4 = TELEMETRY`, `5 = CALIBRATE_MOTORS`, `6 = CALIBRATE_LINE`.
   - Propósito del Force: Si no es forzado, la propia máquina de estados verificará preventivamente si se cumplen las condiciones de sensores y enlace de red requeridos y abortará si es inseguro. Si es `true`, saltará estas salvaguardias.
   - Respuesta Éxito: `{"action": "set_mode", "status": "success", "message": "Mode changed"}` (Asincronamente la MQ emitirá también un evento `MODE_CHANGE` a `robot/events`).
   - Respuesta Fallo: `{"action": "set_mode", "status": "error", "message": "Mode change rejected"}` (P.ej por falta de interlocks o sensores no listos).
