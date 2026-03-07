# ES8311 Codec (espressif__es8311)

## Propósito Arquitectónico
Driver de terceros para controlar de forma fina el hardware es8311, un codec de audio de bajo consumo. Permite transformar flujo I2S digital en pulsos analógicos de salida y digitalizar el input del micrófono de manera síncrona.

## Entorno y Dependencias
Depende estrictamente de los buses físicos integrados del ESP32 (I2C e I2S) y de la interfaz HAL de FreeRTOS/ESP-IDF para la comunicación entre hilos y periféricos.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Bus I2C de control (Dirección 0x18 o 0x19 dependiente del pin CE). Bus I2S bidireccional para el flujo de datos ininterrumpidos. Señales de sincronización (MCLK, SCLK, LRCK).
- **Software:** Inicialización con `es8311_init()` definiendo resoluciones, frecuencias (ej. 44100Hz o 16kHz) de MCLK. Ajustes transparentes en vivo de volumen, muting, y ganancia pasiva (Fade in/out configurables en micro-pasos).

## Flujo de Ejecución Lógico
Se configura en el arranque tras levantar los buses del ESP32. Debe mandarse una secuencia estricta de registros iniciales al chip vía I2C; una vez configurado y sin errores mutuos de reloj MCLK, el sistema simplemente bombea DMA vía I2S hacia/desde el dispositivo de hardware sin requerir atención en bucle de control principal.

## Funciones Principales y Parámetros
- `es8311_init(es8311_handle_t dev, const es8311_clock_config_t *const clk_cfg, const es8311_resolution_t res_in, const es8311_resolution_t res_out)`: Inicializa el codec ES8311.
  - `dev`: Handle I2C del codec instanciado por `es8311_create`.
  - `clk_cfg`: Puntero a la configuración de reloj (MCLK, SCLK, frecuencia de muestreo).
  - `res_in` / `res_out`: Resoluciones de entrada y salida (16, 18, 20, 24, 32 bits).
- `es8311_create(const i2c_port_t port, const uint16_t dev_addr)`: Crea el objeto I2C en memoria devolviendo un `es8311_handle_t`.
- `es8311_voice_volume_set(es8311_handle_t dev, int volume, int *volume_set)`: Ajusta el volumen del DAC.
  - `dev`: El manejador del codec.
  - `volume`: Nivel deseado de 0 a 100.
  - `volume_set`: Puntero de retorno (puede ser NULL) donde inyecta el volumen real aceptado.
- `es8311_voice_mute(es8311_handle_t dev, bool enable)`: Silencia la salida.
- `es8311_microphone_gain_set(es8311_handle_t dev, es8311_mic_gain_t gain_db)`: Configura la ganancia del micrófono (ADC) usando enum de -1 a 42 dB.

## Puntos Críticos y Depuración
- **Reloj Inestable:** Todo sonido dependerá enteramente de la pureza de MCLK generado. Si ESP-IDF o el software cambian las divisiones APLL dinámicamente, generarán chasquidos, agudos, o caídas completas del audio (silencio total o Under-run repetitivo).
- **Dirección I2C Externa:** El ES8311 tiene pines de "Chip Enable" que varían la dirección de I2C. Mal alambrado bloqueará completamente el componente sin mostrar actividad I2C.
