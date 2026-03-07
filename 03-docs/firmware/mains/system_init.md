# System Initialization (system_init)

## Propósito Arquitectónico
Capa de abstracción (Facade) que aísla toda la purga cruda y verbosidad ruidosa de "poner en marcha el metal" y las librerías Core de ESP-IDF fuera del archivo principal (`app_main.c`), logrando de paso una secuencia inalterable y limpia de arranque que puede diagnosticar logs y fallos físicos.

## Entorno y Dependencias
Toca fuertemente y directamente librerías profundas y dispares del framework de bajo nivel que ningún otro programa debiera lidiar individualmente: Non-Volatile Storage (`nvs_flash.h`), Controladores de logs (`esp_log.h`), Pila IP base abstraca (`esp_netif.h`) y audio/interfaces compartidas.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Interfaz NVS Flash para persistencia local del Silicio.
- **Software:** API agresiva de `esp_log_level_set` alterando configuraciones globales del kernel al vuelo para todos los sistemas del chip antes de spawnear nada.

## Flujo de Ejecución Lógico
Silencia primero todas las trazas "Spam" internas no críticas de subsistemas de IDF como I2C, Glue Ethernet y sub-módulos `WARN`, reservando el modo INFO estricto únicamente para Ethernet nativo y Logs Propios de CPU1/Main.
Lanza después módulos fundamentales atómicos (Kernel/Network base). Chequea estado del codec/ethernet vía `ESP_ERROR_CHECK()` o simples `if`, y hace que la placa pite en su primera fase viva de Hardware emitiendo el sonido de arranque (`STARTUP`) mediante `audio_player_play`.

## Funciones Principales y Parámetros
- `system_init(void)`: Punto ciego llamado externamente. Lanza NVS, Netif, MAC/PHY Ethernet, Codec Audio ES8311, e inicializa Mutex y memorias del bloque Central `shared_memory_init()`. Sin parámetros ni retorno explícito, pero interrumpe agresivamente la traza si un `ESP_ERROR_CHECK` falla en Ethernet/NVS (Detiene máquina).

## Puntos Críticos y Depuración
- **Ocultación Ciega de Fallos Reales (Logs):** Al forzar explícitamente `esp_log_level_set("i2c", ESP_LOG_ERROR);` o ahogar logs primitivos de Ethernet e IDFs en los rangos tempranos, si existe posteriormente un fallo eléctrico silencioso de interrupción subyacente en el ESP32, será extremadamente difícil rastrearlo a menos de volver aquí y comentar el silenciador, debiendo alertar a constructores al respecto.
