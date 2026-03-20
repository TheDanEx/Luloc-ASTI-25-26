# INA226 Power Monitor

## Propósito Arquitectónico

El componente `ina226_sensor` es el encargado de la monitorización de potencia del robot Lurloc-ASTI. Proporciona lecturas precisas de voltaje de bus (batería), corriente y consumo de potencia en tiempo real.

Este componente ha sido refactorizado para ofrecer una **API simplificada y eficiente** que permite obtener todas las métricas y gestionar alertas en una sola línea de código, manteniendo el desacoplamiento y la legibilidad.

## Entorno y Dependencias

- **Hardware:** Sensor INA226 conectado vía I2C (Bus 0).
- **Firmware:** ESP-IDF v5.x.
- **Dependencias locales:**
  - `driver`: Requerido para la comunicación I2C (API Legacy).
  - `audio_player`: Utilizado para la emisión de alertas sonoras.

## Interfaces de E/S (Inputs/Outputs)

- **Inputs:**
  - Registros de hardware del INA226 vía I2C.
  - Configuración vía Kconfig (Thresholds y sonidos).
- **Outputs:**
  - Estructura `ina_data_t` con voltaje, corriente y potencia.
  - Alertas auditivas configurables.

## Flujo de Ejecución Lógico

1. **Inicialización:** `ina_init()` realiza el handshake I2C y calibra el chip.
2. **Lectura Inteligente:** `ina_read()` permite capturar todos los datos y procesar alertas automáticamente si se desea, simplificando los bucles de monitoreo.
3. **Alertas:** Se gestionan internamente con un cooldown de 30s para evitar saturar el audio.

## Funciones Principales (API Simplificada)

- `esp_err_t ina_init(void)`: Inicialización y calibración.
- `esp_err_t ina_read(ina_data_t *data, bool check_alerts)`: **Función recomendada.** Lee todo y opcionalmente comprueba alertas.
- `esp_err_t ina_check_alerts(void)`: Comprobación manual de umbrales.
- `esp_err_t ina_read_voltage(float *v_mv)` / `ina_read_current(...)` / `ina_read_power(...)`: Lecturas individuales.

## Ejemplo de Uso (Simplificado)

```c
#include "ina226_sensor.h"

void monitor_task(void *arg) {
    ina_init();
    while(1) {
        ina_data_t pwr;
        // Lee voltios, amperios, vatios y gestiona alertas en UN paso
        if (ina_read(&pwr, true) == ESP_OK) {
            printf("Bat: %.1fV, Consumo: %.1fA\n", pwr.voltage_mv/1000, pwr.current_ma/1000);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
