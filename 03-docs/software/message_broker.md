# Message Broker (Mosquitto)

## Propósito Arquitectónico
Servidor principal del bus de mensajes usando el protocolo Ligero MQTT. Centraliza las peticiones distribuyendo paquetes Publish/Subscribe entre los microcontroladores embebidos (ESP32-P4), contenedores de ROS locales, y sistemas telemétricos de alto nivel (Telegraf).

## Entorno y Dependencias
Usa la imagen pública `eclipse-mosquitto:2` estable. Depende de un volumen de solo-lectura `06-mqtt-broker/mosquitto:/mosquitto/config:ro` que almacena sus reglas `mosquitto.conf`.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Red local estándar e interfaz de puente intra-docker o host IP.
- **Software:** Anuncia servicios explícitamente exponiendo el puerto estándar estático `1883:1883` hacia la red Host del SBC (vital para que ESP32 se asocie a la IP real en la red WiFi/Ethernet). 

## Flujo de Ejecución Lógico
Demonio de latencia ultra-baja y bloqueante I/O en red. Simplemente empareja las colas de las String suscritas a String proveídas. Es la base cero del robot, debe iniciar antes que aplicaciones downstream o las demás pueden ahogarse intentando reconexión perpetua.

## Parámetros de Configuración y Entorno (Docker)
- `ports`: `"1883:1883"`. Se exterioriza estrictamente el puerto sobre el puente de Docker. Mapeado universal y sin cifrado TLS aparente (MQTT puro local).
- `volumes`: Montaje *Read-Only* (`:ro`) desde `./06-mqtt-broker/mosquitto` hacia `/mosquitto/config`. Este directorio aloja explícitamente el archivo `mosquitto.conf` que el binario interno de eclipse usa como maestro al evaluar ACLs (Controles de Acceso) y Listeners.
- `restart`: `unless-stopped`. Se asegura que inicie obligatoriamente al proporcionar corriente a la SBC (arranque OS Host).

## Puntos Críticos y Depuración
- **Memory Footprint:** Si el suscriptor es deficiente (ESP32 con red temporalmente muerta usando QoS 1 o QoS 2), Mosquitto empezará a almacenar internamente los menajes "Retained" o encolados encolapsando la RAM hasta morir si dura varios minutos (MemLeak Virtual).
- **Problemas de autenticación o Bind:** Si el fichero `.conf` no está correctamente seteando `listener 1883` y `allow_anonymous true/false`, el bróker rechazará toda conexión entrante del exterior del entorno Docker.
