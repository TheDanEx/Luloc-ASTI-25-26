# Ethernet

## Propósito Arquitectónico
Maneja el ciclo de vida del enlace cableado de red local en el ESP32-P4. Abstrae la inicialización MAC/PHY, la puesta en marcha de LwIP y el controlador de red.

## Entorno y Dependencias
Dependencia profunda del core de red del ESP-IDF (`esp_netif.h`, `esp_eth_driver.h`) y de la librería LwIP (TCP/IP). La arquitectura parece soportar configuraciones base `example_eth_init()` como wrapper generalizado.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Bus RMII/SMI hacia el PHY Ethernet.
- **Software:** Se notifica al resto del sistema si la red está disponible invocando a `ethernet_is_connected()`.

## Flujo de Ejecución Lógico
Arranque central tras NVS. Lanza event handlers de conectividad y mantiene una máquina de estados de red paralela. Una vez despachado el driver subyacente (`esp_eth_driver`), funciona enteramente guiado por interrupciones de hardware e interfaces LwIP en otra tarea del sistema.

## Funciones Principales y Parámetros
- `ethernet_init(void)`: Función de alto nivel que inicializa toda la pila de red (LwIP, interfaces, handlers) y arranca la máquina de estados del driver Ethernet. No recibe parámetros. Devuelve `ESP_OK` en éxito.
- `ethernet_deinit(void)`: Detiene y limpia la máquina de estados, el driver y la interfaz de red para ahorrar energía o reiniciar la comunicación.
- `ethernet_is_connected(void)`: Función consultiva para conocer el estado estricto del enlace.
  - Devuelve `true` (bool) si el cable está conectado y LwIP ha obtenido una IP local válida.
- `example_eth_init(...)` / `example_eth_deinit(...)`: Funciones wrapper de bajo nivel de ESP-IDF para instanciar/destruir directamente los periféricos MAC/PHY.

## Puntos Críticos y Depuración
- **Desconexión Súbita:** Si el cable se desconecta, se deben limpiar conexiones MQTT y sockets para re-establecimiento automático.
- **Auto-Nego Fallida:** Error grave en la inicialización si el PHY no reporta link. Depende extremadamente de un correcto pin-mapping y reloj de 50Mhz de RMII persistente.
