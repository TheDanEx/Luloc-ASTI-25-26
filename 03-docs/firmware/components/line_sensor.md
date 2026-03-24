# Component: line_sensor

## Propósito Arquitectónico
Este componente abstrae el array de 8 sensores de línea del robot Lurloc-ASTI. Gestiona la lectura mixta de entradas digitales (D1, D2, D7, D8) y analógicas (D3, D4, D5, D6), aplicando sobremuestreo para mitigar el ruido del ADC del ESP32-P4 y proporcionando una interfaz normalizada para algoritmos de control.

## Entorno y Dependencias
- **Hardware:** Requiere 4 pines GPIO digitales y 4 pines ADC (Uni 1).
- **Firmware:** Depende de `driver/gpio` y `esp_adc/adc_oneshot`.

## Interfaces de E/S (Inputs/Outputs)
- **Inputs:** Señales crudas de los sensores (0/1 digital, 0-4095 analógico).
- **Outputs:** Valores normalizados (0.0-1.0), binarizados (0/1) y posición calculada en mm.

## Flujo de Ejecución Lógico
1. **Init:** Configura GPIOs y el ADC unit.
2. **Read Raw:** Itera sobre los sensores. Los analógicos se leen $N$ veces y se promedian.
3. **Normalize:** Escala los valores analógicos entre `min_value` y `max_value` configurados.
4. **Position:** Calcula la media ponderada de los valores normalizados usando los pesos (distancia en mm) de cada sensor.

## Funciones Principales y Parámetros
- `line_sensor_init()`: Inicialización de hardware.
- `line_sensor_read_raw(uint32_t *raw, uint32_t samples)`: Lectura física.
- `line_sensor_read_line_position()`: Retorna la posición en mm respecto al centro.

## Puntos Críticos y Depuración
- **Ruido ADC:** El ESP32-P4 requiere al menos 16 muestras de sobremuestreo para lecturas estables en entornos industriales.
- **Calibración:** Los valores Min/Max deben ajustarse según la superficie de la pista.

## Ejemplo de Uso e Instanciación
```c
line_sensor_init();
float pos = line_sensor_read_line_position();
ESP_LOGI("APP", "Linea en: %.2f mm", pos);
```
