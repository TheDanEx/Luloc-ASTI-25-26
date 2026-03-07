# Vision Node (vision_node)

## Propósito Arquitectónico
Provee el pipeline principal de procesamiento de imágenes para el robot Lurloc-ASTI. Presumiblemente procesa tramas de cámaras locales (USB o CSI) mediante OpenCV u otros motores, para detección de objetos, AprilTags o tracking en tiempo real.

## Entorno y Dependencias
Contenedor basado en ROS 2 (deducido por variables como `ROS_DOMAIN_ID=13`). Integración completa con Docker Compose montando volúmenes de desarrollo en caliente desde `./01-vision/src`.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Dispositivo lógico `/dev/video0` mappeado directamente dentro del contenedor sin restricciones (`privileged` implícito al inyectar dev o en el config de build).
- **Software:** Usa Tópicos ROS 2 para publicación (Ej. `sensor_msgs/Image`, `geometry_msgs/Twist`) o Topics MQTT puenteados indirectamente. Usa el `network_mode: host` para evitar latencias de NAT y compartir puertos directamente con Nav/Telemetría.

## Flujo de Ejecución Lógico
Lee el archivo del dispositivo V4L2 en bucle constante dictado por el framerate de la cámara. Procesa fotogramas pasando algoritmos de visión, empaca los resultados y coordenadas relativas al map/base_link, y los escupe al middleware de ROS (DDS). Se reinicia en caso de choque continuo de la tarea (`unless-stopped`).

## Parámetros de Configuración y Entorno (Docker)
- `ROS_DOMAIN_ID`: Identificador de subred DDS (actualmente `13`). Aisla el tráfico ROS espacialmente si hay otros robots en la red WiFi general.
- `devices`: `/dev/video0:/dev/video0`. Mapeo directo y passthrough del hardware (/dev/video) hacia el contenedor asumiendo compatibilidad V4L2 en el kernel anfitrión.
- `network_mode: "host"`: Omitido el ruteo interno bridge de docker, la aplicación usa las verdaderas IPs y puertos de la máquina SBC host.
- `volumes`: Montaje bidireccional local (`./01-vision/src`) mapeado a `/root/vision_node` facilitando edición live (Hot-reloading/scripts).

## Puntos Críticos y Depuración
- **Falta de Hardware /Dumping:** Falla fatal si `/dev/video0` (cámara USB/CSI) se desconecta temporalmente por un bache del robot; el nodo caerá, requiriendo que la directiva de restart lo reviva, pero perdiendo valiosos segundos de control.
- **Cuello de Botella Computacional:** La visión directa en SBC (Raspberry Pi/Similares) satura el CPU rápidamente si las resoluciones y compresión sobre USB se configuran inadecuadamente causando retraso temporal profundo entre fotogramas entrantes y comandos ROS resultantes.
