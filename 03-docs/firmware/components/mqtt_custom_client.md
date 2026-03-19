# MQTT Custom Client

## Propósito Arquitectónico
Garantiza el transporte estándar bidireccional de telemetría y directivas a través de protocolo MQTT bajo tópicos estándar. Adicionalmente, cuenta con un WatchDog asociado para monitorizar salud de eventos.

## Entorno y Dependencias
API de esp-mqtt Client. Está amarrado directamente a la disponibilidad del stack IP que le entregue el módulo `ethernet`.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** (Sólo Red IP)
- **Software:** 
   - Consumo: Callbacks por subscripción. (Tópicos como `robot/api/request`, o telemetría `robot/telemetry/#`).
   - Producción (Logging/Monitoring): `robot/logs/#`, `robot/events` y `robot/telemetry/#` siempre en **Influx Line Protocol (ILP)**.

## Flujo de Ejecución Lógico
Lanza hilo subyacente MQTT. Proceso re-activo por eventos (conectado, desconectado, recibido, data_error). Ofrece un registro a observadores lógicos para que módulos superiores puedan atrapar los payloads en un callback despachado velozmente sin saturar el dispatcher.

## Funciones Principales y Parámetros
- `mqtt_custom_client_init(void)`: Configura y arranca el task MQTT nativo del cliente con las URIs/IPs registradas en KConfig. Debe llamarse cuando la capa IP haya completado su negociación.
- `mqtt_custom_client_publish(const char *topic, const char *data, int len, int qos, int retain)`: Publica data empaquetada al servidor.
  - `topic`: La ruta o Tag del mensaje (Ej. `"robot/telemetry/speed"`).
  - `data`: El payload en crudo a emitir, puede ser JSON, cadena de texto o binario (Protobuf).
  - `len`: Longitud total del payload en bytes. Ingestar `0` permite que la función internamente calcule la longitud si asume que `data` incluye un null-terminator `\0`.
  - `qos`: Nivel Quality of Service (0 = Fire&Forget, 1 = Esperar ACK del Broker).
  - `retain`: Boolean (0 o 1) indicando al bróker si debe guardar este mensaje hasta ser sobreescrito.
- `mqtt_custom_client_log(const char *level, const char *fmt, ...)`: Envía trazas de depuración remota en **formato ILP** con timestamp de ns.
- `mqtt_custom_client_debug(const char *fmt, ...)`: Envía datos crudos de desarrollo al tópico `robot/debug` en formato ILP.
- `mqtt_custom_client_subscribe(const char *topic, int qos)`: Exige al servidor el envío de todos los mensajes que cumplan con la máscara de `topic`.
- `mqtt_custom_client_unsubscribe(const char *topic)`: Anula una suscripción previa limitando el tráfico y ahorrando ancho de banda.
- `mqtt_custom_client_register_topic_callback(const char *topic, mqtt_message_callback_t callback)`: Registra a nivel de software un listener específico en forma abstracta.
  - `topic`: Ruta string suscrita de interés donde deseamos accionar código embebido.
  - `callback`: Puntero a función con el prototipo base `void callback(const char *topic, int topic_len, const char *data, int data_len)`.

## Puntos Críticos y Depuración
- **Retardo extremo en Callbacks:** Todo callback llamado bloquea al task receptor de MQTT, cualquier delay, print, cálculo pesado allí tumbará el client Keep-Alive, resultando en desconexiones cíclicas. Toda carga ha de delegarse a RTOS Event Groups/Queues.
- **WatchDog Trigger:** Fallos de hardware paralelos pueden hacer que la red caiga o sature temporalmente.

## Ejemplo de Uso e Instanciación
```c
#include "mqtt_custom_client.h"

// 1. Callback extremadamente rápido. NO bloquear aquí.
void on_cmd_received(const char *topic, int topic_len, const char *data, int data_len) {
    // Extraer dato e inyectarlo en Tarea RTOS mediante Queue o Flags
    if (strncmp(data, "START", data_len) == 0) {
        xEventGroupSetBits(robot_event_group, BIT_START_MISSION);
    }
}

// 2. Al recibir confirmación de red LwIP (ej. tras ethernet_init)
void network_ready_task(void *pvParameters) {
    // Inicializa motor MQTT (usa IPs de Kconfig) y arranca hilo demonio
    mqtt_custom_client_init();

    // Registrar observadores lógicos en memoria
    mqtt_custom_client_register_topic_callback("robot/cmd", on_cmd_received);

    // Obligar la suscripción al broker IoT (QoS 1 = Asegurar llegada)
    mqtt_custom_client_subscribe("robot/cmd", 1);

    while (1) {
        // Publicación de latidos por rutina (ILP)
        const char* msg = "heartbeat,robot=Lurloc status=1i";
        mqtt_custom_client_publish("robot/telemetry/system", msg, 0, 0, 0);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
