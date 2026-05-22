# System Setup & Prerequisites (Requisitos Previos del Sistema)

Este documento centraliza todas las configuraciones host, dependencias de kernel y ajustes de entorno (`menuconfig`) que **deben** estar preparados en el hardware físico antes de intentar compilar o desplegar el software/firmware del robot Lurloc-ASTI.

## 1. SBC (Raspberry Pi 5 / Ubuntu) - Requisitos Host

Para que los contenedores Docker puedan interactuar directamente con el hardware a bajo nivel (Camaras, Telemetria Termica), el sistema operativo base debe cumplir condicionalmente:

### A. Passthrough de Periféricos (Camara V4L2)
Para el contenedor `vision_node`:
- La cámara CSI o USB debe estar enumerada correctamente como `/dev/video0` en el host.
- El usuario bajo el que corre Docker o el contenedor debe tener acceso a este grupo, o usar `devices: ["/dev/video0:/dev/video0"]`.

---

## 2. Firmware (ESP32-P4) - Requisitos `menuconfig`

Antes de hacer `idf.py build`, asegúrese de preconfigurar el ADN del RTOS a través de la herramienta de configuración de Espressif (`idf.py menuconfig`):

### A. Memoria y Optimizaciones
- **Task Watchdog Timer (TWDT):** Asegurarse de que el WDT está activado (panic upon timeout) para prevenir cuellos de botella silenciosos en el `task_rtcontrol_cpu0`.
- **FreeRTOS Tick Rate:** Debería mantenerse a mínimo 1000 HZ (1 ms por tick) en `Component config > FreeRTOS > Tick rate (Hz)` si buscamos fidelidad temporal en los retrasos de control.
