# mqtt_api_responder

## Propósito Arquitectónico
Este componente actúa como un servidor "REST-Like" sobre MQTT, exponiendo una API Síncrona. A diferencia de la telemetría periódica asíncrona (push), permite a los clientes externos (scripts, dashboards o ingenieros) pedir explícitamente el estado de sensores específicos (queries GET) o inyectar comandos de control accionables (comandos SET) recibiendo un JSON de respuesta estandarizado de forma instantánea. 

## Entorno y Dependencias
Requiere `mqtt_custom_client` para la subscripción de tópicos y publicación JSON de respuesta.
Requiere `shared_memory` para acceder de forma segura (Thread-Safe) a la información en tiempo real del robot (motores, encoders, voltajes) suministrada por la CPU0.
Requiere `cJSON` para el _parsing_ robusto de la mensajería entrante y el _stringifying_ saliente.
Requiere `audio_player` para recibir e inyectar solicitudes de comando SET de audio desde hardware externo a través de la red.

## Interfaces de E/S (Inputs/Outputs)
- **Input (GET):** Se suscribe configurablemente al tópico `robot/api/get` esperando `{"resource": "<nombre>"}`.
- **Input (SET):** Se suscribe configurablemente al tópico `robot/api/set` esperando acciones tipo `{"action": "play_sound", "sound_id": 1}`.
- **Output:** Responde dinámicamente de forma Síncrona publicando el resultado en `robot/api/response` (Configurable vía Kconfig).

## Flujo de Ejecución Lógico
1. Durante la carga, expone la función `mqtt_api_responder_init()` la cual invoca un Topic Callback en el cliente MQTT para los tópicos GET y SET.
2. Tras la conexión (`mqtt_api_responder_subscribe()`), permanece escuchando de forma completamente asíncrona dentro de la Tarea principal de Comms en la CPU1.
3. Al recibir un JSON válido, parsea las llaves `resource` (para consultas) y `action` (para órdenes).
4. Genera al vuelo un objeto `cJSON` con la respuesta estructurada tomando los bloqueos Mutex correspondientes de `shared_memory`, publica en `robot/api/response`, y libera (free) la memoria dinámica cJSON previniendo fugas.

## Funciones Principales y Parámetros
- `esp_err_t mqtt_api_responder_init(void)`: Registra el Callback de tópicos con el subsistema superior. Retorna `ESP_OK` o error de dependencias.
- `esp_err_t mqtt_api_responder_subscribe(void)`: Ejecuta explícitamente el Subscribe hacia los tópicos definidos por Kconfig tras confirmarse un link vivo TCP/IP-MQTT.

## Puntos Críticos y Depuración
- **Memory Leaks:** Al gestionar `cJSON_ParseWithLength` y `cJSON_CreateObject`, se garantiza que en cualquier salida prematura por sintaxis o tipo de Action erróneo se llama un bloque exhaustivo de `cJSON_Delete`.
- **Race Conditions:** La recolección de `battery` o `encoder` accede a `shared_memory_get()`, por lo que está sometida a latencias de Mutex si la CPU0 se queda colgada más de 100ms.

## Ejemplo de Uso e Instanciación

Dentro de la tarea de la CPU1 de comunicaciones:

```c
#include "mqtt_api_responder.h"

// Después de arrancar el cliente base MQTT:
if (mqtt_custom_client_init() == ESP_OK) {
    if (mqtt_api_responder_init() == ESP_OK) {
        ESP_LOGI(TAG, "API Responder listo. Esperando Link para Subscribe...");
    }
}

// Durante el bucle 1Hz u observador de red
if (mqtt_custom_client_is_connected()) {
    mqtt_api_responder_subscribe();
}
```
