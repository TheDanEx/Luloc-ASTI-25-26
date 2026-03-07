# PTP IEEE-1588 Configuration & Documentation

Para que el ecosistema de sincronización estricta (<1ms) PTP funcione con la Raspberry Pi 5 y el ESP32-P4, deben cumplirse **obligatoriamente** los siguientes requisitos en sus respectivos sistemas:

## Lado Red / Docker (Raspberry Pi 5)
El demonio `linuxptp` inyectado en docker requiere privilegios del Kernel en el anfitrión:
1. Asegurarse de que el kernel de la RPi tiene habilitado `CONFIG_NETWORK_PHY_TIMESTAMPING`.
2. Dentro del `docker-compose.yml`, el servicio DEBE usar `network_mode: "host"`.
3. DEBE poseer las capacidades (Capabilities) de Docker `SYS_TIME` y `NET_ADMIN`.
   - `SYS_TIME`: Cambia la hora del SO.
   - `NET_ADMIN`: Solicita a los sockets L2 operaciones de Timestamping por hardware.

## Lado Firmware ESP-IDF (ESP32-P4)
En la configuración global del menú del firmware (`idf.py menuconfig`), aplicar:
1. **Component config > LWIP > Enable ESP LwIP IPv4/IPv6 PTP/SNTP (u opciones similares de multicast).**
    - El socket PTP debe unirse al grupo multicast base `224.0.1.129`. LwIP requiere que IGMP esté habilitado en su stack.
2. **Component config > Ethernet > Enable Hardware Timestamping:**
    - Si el EMAC del ESP32-P4 y el chip PHY lo soportan nativamente, marcar el Hardware Timestamping para evadir la latencia de interrupción de software (ISR latency) que el RTOS induce. Si no, forzar Software Timestamping en LwIP Socket Options (por ej. `SOF_TIMESTAMPING_SOFTWARE`).
3. El componente desarrollado `ptp_client` debe ser inicializado ÚNICAMENTE después de que `ethernet.c` haya emitido el evento `IP_EVENT_ETH_GOT_IP`. De lo contrario el _bind_ del socket UDP fallará irrecuperablemente.
