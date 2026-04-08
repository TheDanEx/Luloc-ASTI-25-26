# PTP Client (IEEE-1588)

## Propósito Arquitectónico
Implementa un cliente de Precision Time Protocol (IEEE-1588) sobre UDP Multicast para sincronización horaria de nivel industrial. Permite que el sistema de telemetría del ESP32 estampe las tramas de InfluxDB con precisión sub-milisegundo basándose en un Grandmaster de red local (ej. Raspberry Pi).

## Entorno y Dependencias
Depende de `lwip` para sockets POSIX, de la activación estricta de `CONFIG_LWIP_IGMP=y` para ruteo Multicast, y de `esp_timer.h` para interpolación microscópica de ticks libres. Dispone de `esp_sntp` para fallbacks fijos.

## Interfaces de E/S (Inputs/Outputs)
- **Inputs:** Paquetes UDP Multicast en la IP `224.0.1.129` (Puerto `320` para mensajes Follow_Up PTP de 2 Pasos).
- **Outputs:** Exporta la API global `get_ptp_timestamp_us()` para inyectar la fecha Epoch asíncrona perfecta.
- **Output (ASYNC):** Al bloquear la sincronía válida emite un evento `TIME_SYNC` (formato Influx Line Protocol) al tópico `robot/events`.

## Flujo de Ejecución Lógico
1. `ptp_client_init()` lanza el Listener UDP Multicast de baja prioridad.
2. Al recibir un paquete UDP en el puerto `320` (`msg_id == 0x08`), parsea los bytes del Epoch PTP entrantes del Grandmaster con el valor absoluto del silicio local en ese mismo instante (`esp_timer_get_time()`) fijando un Offset Base.
3. El módulo de Telemetría invoca a `get_ptp_timestamp_us()`, sumando el contador local al Offset de la red para conseguir el log 100% puro en nanosegundos sin demoras de software.

## Funciones Principales y Parámetros
- `ptp_client_init()`: Crea el Task interno del Listener.
- `get_ptp_timestamp_us()`: API estática de extracción Unix Epoch del sistema global (en Microsegundos).
- `ptp_is_synchronized()`: Alerta local de sincronía.
- `ptp_client_force_sync()`: Fuerza la bajada del flag síncrono y reinicia el servicio SNTP de apoyo para forzar al sistema a re-capturar el offset desde cero.

## Puntos Críticos y Depuración
- **Recepción IGMP Cegada:** Si Wi-Fi o ETH actúan sordiamente a Multicast, compruebe siempre `CONFIG_LWIP_IGMP=y` en Menú Kconfig Base.
- **Topología de Reloj (2-Step PTP):** Escucha expresamente el Puerto 320 (`Follow_Up`). La mayoría de Master Clocks hardware/software (`ptp4l` con `-H` o `-S`) usan doble paso delegando el Epoch al mensaje de seguimiento, el System inicial de Sync (Puerto 319) emite reloj cero y se ignora matemáticamente.

## Ejemplo de Uso e Instanciación
```c
#include "ptp_client.h"

// 1. Inicialización Post-DHCP
ptp_client_init();

// 2. Extraer tiempos 
int64_t high_res_epoch = get_ptp_timestamp_us();
```
