# Componente: Power Monitor (`ina219_sensor`)

## Propósito Arquitectónico
Este componente abstrae el acceso de bajo nivel por I2C al sensor de corriente/voltaje INA219. Su objetivo principal es monitorizar la tensión y el consumo de corriente (miliamperios y milivoltios) a nivel global de la batería o del bus de motores. Se configura estáticamente a través del `menuconfig` (Kconfig).
En la arquitectura del robot, se recomienda que este sensor sea interpelado por el núcleo de bajo consumo (`task_monitor_lowpower_cpu1`), el cual almacena los resultados en `shared_memory` de inmediato para el núcleo base de RTOS, y delega los envíos sobre InfluxDB/MQTT mediante batching en el `telemetry_manager`.

## Entorno y Dependencias
- **Framework:** ESP-IDF v5.5.
- **Hardware:** INA219 (Sensor I2C).
- **Dependencias Locales:** Puesta a punto con `driver/i2c_master.h`, `esp_log.h` y `sdkconfig.h`.
- **Integración con Configuración:** Expone opciones dependientes del shunt (físico), mapeo GPIO (SDA/SCL) y frecuencias I2C customizadas en `Kconfig`.

## Interfaces de E/S (Inputs/Outputs)
### Inputs:
- Resistencia del Shunt (configurado vía *Kconfig* `CONFIG_INA219_SHUNT_OHMS`).
- Corriente máxima esperada (configurado vía *Kconfig* `CONFIG_INA219_MAX_EXPECTED_AMPS`).

### Outputs (Funciones de Lógica de Negocio):
- `voltage_mv`: Voltaje crudo del bus medido.
- `current_ma`: Corriente consumida derivada de la matriz interna de ganancia-registo-calibración.
- `power_mw`: Potencia consumida.

## Flujo de Ejecución Lógico
1. El usuario inicializa el hardware con `ina219_sensor_init()`. Esta función instancia dinámicamente el bus I2C maestro y susurra a la dirección MAC del bus I2C (0x40).
2. Ocurre la **Calibración Interna:** Lee Kconfig, parsea los strings numéricos configurados en Ohmios y Amperios (ej. 0.01 y 3.2), calcula los Least Significant Bits (`LSB`) en tiempo de ejecución de C, y aplica los registros `Register 05` (Calibración) y `Register 00` (Configuración de modo continuo `MODE_SANDBVOLT_CONTINUOUS`).
3. El usuario puede entonces llamar cíclicamente a `ina219_sensor_read_bus_voltage_mv(...)`, la que pregunta por registro 02 y computa milivoltios.

## Funciones Principales y Parámetros
- `esp_err_t ina219_sensor_init(void)`: Inicializa HW y escribe buffers de calibración I2C.
- `esp_err_t ina219_sensor_read_bus_voltage_mv(float *voltage_mv)`: Lectura de tensión de Bus (precisión 4mV).
- `esp_err_t ina219_sensor_read_current_ma(float *current_ma)`: Lectura de Corriente derivada.
- `esp_err_t ina219_sensor_read_power_mw(float *power_mw)`: Lectura de Potencia.

## Puntos Críticos y Depuración
- **Reinicio Caliente vs Frío:** `ina219_sensor_init` soporta ser llamado repetidamente gracias al chequeo `if (dev_handle != NULL)`, pero previene registrar puertos de I2C duplicados en `i2c_new_master_bus`.
- **Valores Congelados:** Si el voltaje reporta cero constantemente, compruebe que el `CONFIG_INA219_I2C_ADDRESS` es (predeterminado a 0x40) y que los pines SDA y SCL en el ESP32-P4 están cableados debidamente (Kconfig configurado a 21 y 22 por defecto ESP32, modificar dependiendo de tabla GPIO del P4).

## Ejemplo de Uso e Instanciación
```c
#include "ina219_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Normalmente instanciado en main/tasks/task_monitor_lowpower_cpu1.c
void task_monitor_lowpower_cpu1(void *arg) {
    if (ina219_sensor_init() != ESP_OK) {
        ESP_LOGE("PWR", "Error INA219");
    }

    while (1) {
        float f_volts, f_amps, f_milliwats;

        ina219_sensor_read_bus_voltage_mv(&f_volts);
        ina219_sensor_read_current_ma(&f_amps);
        ina219_sensor_read_power_mw(&f_milliwats);

        ESP_LOGI("PWR", "Volts: %.1fmV | Current: %.1fmA", f_volts, f_amps);

        vTaskDelay(pdMS_TO_TICKS(100)); // Polling 10 Hz
    }
}
```
