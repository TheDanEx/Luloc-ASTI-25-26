# MQTT Custom Client

## Propósito Arquitectónico
Garantiza el transporte estándar bidireccional de telemetría y directivas a través de protocolo MQTT bajo tópicos estándar. Adicionalmente, cuenta con un WatchDog asociado para monitorizar salud de eventos.

## Entorno y Dependencias
API de esp-mqtt Client. Está amarrado directamente a la disponibilidad del stack IP que le entregue el módulo `ethernet`.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** (Sólo Red IP)
- **Software:** 
   - Producción: `mqtt_custom_client_publish()` con control semántico de QoS y persistencia (Retain).
   - Consumo: Callbacks por subscripción y delegación estricta de memoria al suscriptor. (Tópicos como `robot/cmd`, o telemetría `robot/telemetry`).

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
  - Retorna numéricamente el `message_id`.
- `mqtt_custom_client_subscribe(const char *topic, int qos)`: Exige al servidor el envío de todos los mensajes que cumplan con la máscara de `topic`.
- `mqtt_custom_client_unsubscribe(const char *topic)`: Anula una suscripción previa limitando el tráfico y ahorrando ancho de banda.
- `mqtt_custom_client_register_topic_callback(const char *topic, mqtt_message_callback_t callback)`: Registra a nivel de software un listener específico en forma abstracta.
  - `topic`: Ruta string suscrita de interés donde deseamos accionar código embebido.
  - `callback`: Puntero a función con el prototipo base `void callback(const char *topic, int topic_len, const char *data, int data_len)`.

## Puntos Críticos y Depuración
- **Retardo extremo en Callbacks:** Todo callback llamado bloquea al task receptor de MQTT, cualquier delay, print, cálculo pesado allí tumbará el client Keep-Alive, resultando en desconexiones cíclicas. Toda carga ha de delegarse a RTOS Event Groups/Queues.
- **WatchDog Trigger:** Fallos de hardware paralelos pueden hacer que la red caiga o sature temporalmente.
