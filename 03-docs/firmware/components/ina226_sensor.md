# INA226 Power Monitor

## Propósito Arquitectónico

El componente `ina226_sensor` es el encargado de la monitorización de potencia del robot Lurloc-ASTI. Proporciona lecturas precisas de voltaje de bus (batería), corriente y consumo de potencia en tiempo real.

Este componente está optimizado para el chip **INA226**, que ofrece una resolución superior (1.25mV por LSB de voltaje) comparado con modelos anteriores como el INA219.

## Entorno y Dependencias

- **Hardware:** Sensor INA226 conectado vía I2C (Bus 0).
- **Firmware:** ESP-IDF v5.x.
- **Dependencias locales:**
  - `esp_driver_i2c`: Requerido para la comunicación física.
  - `audio_player`: El sensor comparte el bus I2C con el codec de audio, por lo que depende de que el driver I2C haya sido inicializado previamente por el componente de audio.

## Interfaces de E/S (Inputs/Outputs)

- **Inputs:**
  - Registros de hardware del INA226 vía I2C.
  - Configuración vía Kconfig (`Shunt value`, `Expected Current`).
- **Outputs:**
  - `voltage_mv`: Voltaje de la batería en milivoltios.
  - `current_ma`: Corriente de consumo en miliamperios.
  - `power_mw`: Potencia consumida en milivatios.
  - Logs de depuración en consola (`INA226`).

## Flujo de Ejecución Lógico

1. El sensor se inicializa llamando a `ina226_sensor_init()`.
2. Se verifica el ID del fabricante (0x5449) para asegurar la presencia del chip INA226.
3. Se realiza una calibración interna calculando el valor del registro `CALIBRATION` basado en el Shunt y la Corriente Máxima esperada definidos en Kconfig.
4. Las funciones de lectura acceden directamente a los registros de datos, aplicando los factores de escala (LSBs) adecuados.

## Funciones Principales y Parámetros

- `esp_err_t ina226_sensor_init(void)`: Inicializa y calibra el chip.
- `esp_err_t ina226_sensor_read_bus_voltage_mv(float *voltage_mv)`: Obtiene el voltaje de bus. LSB = 1.25mV.
- `esp_err_t ina226_sensor_read_current_ma(float *current_ma)`: Obtiene la corriente.
- `esp_err_t ina226_sensor_read_power_mw(float *power_mw)`: Obtiene la potencia.

## Puntos Críticos y Depuración

- **Coexistencia I2C:** Al compartir el bus con el audio, cualquier error en el driver I2C afectará a ambos. Se debe asegurar que `audio_player_init` se ejecute antes.
- **Precisión:** La lectura de voltaje es nativamente 1.25mV/bit. Si se detecta una desviación constante contra multímetro, verificar la integridad del chip o posibles caídas de tensión en las pistas del PCB.

## Ejemplo de Uso e Instanciación

```c
#include "ina226_sensor.h"

void app_main(void) {
    // Inicializar audio primero (levanta el bus I2C)
    audio_player_init();

    // Inicializar sensor de potencia
    if (ina226_sensor_init() == ESP_OK) {
        float v_bat = 0;
        ina226_sensor_read_bus_voltage_mv(&v_bat);
        printf("Batería: %.2f V\n", v_bat / 1000.0f);
    }
}
```
