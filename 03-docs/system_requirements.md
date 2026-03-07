# System Setup & Prerequisites (Requisitos Previos del Sistema)

Este documento centraliza todas las configuraciones host, dependencias de kernel y ajustes de entorno (`menuconfig`) que **deben** estar preparados en el hardware físico antes de intentar compilar o desplegar el software/firmware del robot Lurloc-ASTI.

## 1. SBC (Raspberry Pi 5 / Ubuntu) - Requisitos Host

Para que los contenedores Docker puedan interactuar directamente con el hardware a bajo nivel (Cámaras, Relojes PTP, Telemetría Térmica), el sistema operativo base debe cumplir condicionalmente:

### A. Capacidades PTP (Precision Time Protocol)
Para actuar como *Grandmaster Clock* hacia el ESP32:
- El Kernel Linux del Host **debe** tener habilitado el soporte de Timestamping por Hardware en la tarjeta MAC Ethernet (`CONFIG_NETWORK_PHY_TIMESTAMPING`).
- Se puede verificar corriendo `ethtool -T eth0` en la terminal del Host. Debe listar `SOF_TIMESTAMPING_TX_HARDWARE` y `SOF_TIMESTAMPING_RX_HARDWARE`.
- Los contenedores Docker que utilicen `ptp4l` necesitan obligatoriamente ser lanzados con las capacidades: `cap_add: [SYS_TIME, NET_ADMIN]` y `network_mode: "host"`.

### B. Passthrough de Periféricos (Cámara V4L2)
Para el contenedor `vision_node`:
- La cámara CSI o USB debe estar enumerada correctamente como `/dev/video0` en el host.
- El usuario bajo el que corre Docker o el contenedor debe tener acceso a este grupo, o usar `devices: ["/dev/video0:/dev/video0"]`.

---

## 2. Firmware (ESP32-P4) - Requisitos `menuconfig`

Antes de hacer `idf.py build`, asegúrese de preconfigurar el ADN del RTOS a través de la herramienta de configuración de Espressif (`idf.py menuconfig`):

### A. Precision Time Protocol (PTP IEEE-1588)
El cliente `ptp_client` del ESP32 fallará silenciosamente si las capas bajas de red no se preparan:
- **Enable ESP LwIP PTP Multicast:** Navegar a `Component config > LWIP` y habilitar las opciones referentes a `IPv4/IPv6 PTP/SNTP` e interceptación de tráfico Multicast (`IGMP`). El grupo vital a escuchar es `224.0.1.129`.
- **Hardware Timestamping EMAC:** Navegar a `Component config > Ethernet`. Habilitar el estampe de tiempo por hardware puro si el MAC del ESP32-P4 lo soporta para evadir *Jitter* del RTOS. Si no dispone de HW-Timestamp, forcemos el Software Timestamping genérico en las opciones de LwIP.

### B. Memoria y Optimizaciones
- **Task Watchdog Timer (TWDT):** Asegurarse de que el WDT está activado (panic upon timeout) para prevenir cuellos de botella silenciosos en el `task_rtcontrol_cpu0`.
- **FreeRTOS Tick Rate:** Debería mantenerse a mínimo 1000 HZ (1 ms por tick) en `Component config > FreeRTOS > Tick rate (Hz)` si buscamos fidelidad temporal en los retrasos de control.
