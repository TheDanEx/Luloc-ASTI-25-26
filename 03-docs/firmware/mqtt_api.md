# Robot ESP32 - Documentación de API MQTT Genérica

Este documento consolida la totalidad de tópicos y payloads (Mensajes) que maneja el cerebro robótico ESP32 a través del protocolo MQTT. El sistema utiliza MQTT tanto de forma Asíncrona (Telemetría y Alertas) como de forma Síncrona (Peticiones JSON al vuelo y Settings NVS).

## 1. Comandos de Reducción Simples (Inputs Asíncronos)
*Gestionado por: `task_comms_cpu1` (RX).*

El microcontrolador escucha órdenes nativas literales tipo String para cambiar sus estados internos a nivel global o disparar actuadores tontos.
**Tópico Base:** `robot/cmd`

| Payload Esperado (String Mínimo) | Descripción de la Acción |
| :--- | :--- |
| `CMD_MODE:<n>` | Fuerza la máquina de estados a cambiar a la Fase `n` (siendo `n` un ENUM). |
| `MODE_AUTONOMOUS` | Solicita cambiar específicamente a Conducción Autónoma pura. |
| `MODE_REMOTE_DRIVE` | Solicita cambiar específicamente a Pilotaje Remoto por Gamepad. |
| `MODE_TELEMETRY_STREAM` | Solicita cambiar a modo de Inspección (Motores parados, pero Telemetría al 100%). |
| `CMD_PLAY_SOUND:<id>:<vol>` | Solicita al chip ES8311 que reproduzca el sonido de la SD de id `<id>` a volumen `%vol`. Si se omite `<vol>`, usa el por defecto. |

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
   - Respuesta: `{"resource": "battery", "battery_mv": 16400, "motor_current_ma": 420.5}`
2. Petición Cinemática: 
   - Envío: `{"resource": "encoder"}`
   - Respuesta: `{"resource": "encoder", "speed_ms": 1.2, "encoder_ticks": 454322}`
3. Petición Tiempos: 
   - Envío: `{"resource": "uptime"}`
   - Respuesta: `{"resource": "uptime", "uptime_sec": 3491, "uptime_ms": 3491500}`
4. Petición Total: 
   - Envío: `{"resource": "all"}`
   - Respuesta: `{"resource": "all", "battery_mv": 16400, "motor_current_ma": 420.5, "speed_ms": 1.2, "encoder_ticks": 454322, "uptime_sec": 3491, "uptime_ms": 3491500}`
   
   *(Si un `resource` no existe en el código C del firmware, el robot retornará amistosamente un JSON informando `{"resource": "foo", "error": "Unknown Resource"}`).*
