# Test Sensor

## Propósito Arquitectónico
Instrumental base que demuestra el uso del flujo de lectura desde la placa. Sirve esencialmente para devolver métricas temporales seguras (Uptime) usando reloj de Hardware y comprobar la vitalidad in-interrumpida del microcontrolador.

## Entorno y Dependencias
API nativa del timer del ESP32 (`esp_timer.h` de ESP-IDF).

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** El temporizador interno de 64 bits del microcontrolador. No depende de pines.
- **Software:** Proporciona un puntero transparente con estructura auto-explicativa (`test_sensor_data_t`) con medidas escaladas (ms, seg, min) o una función de impresión `test_sensor_get_uptime_str()`.

## Flujo de Ejecución Lógico
Funciona independientemente pidiendo valores pasivos no-bloqueantes cada vez que el programa superior lo interpela (`test_sensor_read()`). Es completamente thread-safe a nivel lectura ya que no depende de colas ni pre-renders asíncronos en back.

## Funciones Principales y Parámetros
- `test_sensor_init(void)`: Inicia el driver de reloj local si no existía pre-inicializado. Retorna ESP_OK.
- `test_sensor_read(test_sensor_data_t *data)`: Vuelca mediciones en una trama temporal estructurada de diferentes magnitudes (ms, seg, min).
  - `data`: Puntero vacío a estructura del llamador, en donde sobreescribirá dichas métricas.
- `test_sensor_get_uptime_ms(void)` y `_sec(void)`: Obtenciones rápidas que puentean a tipos nativos base de entero escalar de ESP-IDF abstraídas sin necesidad de padding.
- `test_sensor_get_uptime_str(char *buffer, size_t buffer_size)`: Escribe un formateo legíble (Ej: `"12h 45m 0s"`) directamente en el stack de quién hace el logreo local o visual.

## Puntos Críticos y Depuración
- Prácticamente exento de fallos. El uso de `esp_timer_get_time()` previene desbordamientos de 32 bits tradicionales en `delay()` de Arduino (Tardaría centurias en sobrepasar los 64bits reales).
