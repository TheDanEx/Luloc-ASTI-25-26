# Audio Player

## Propósito Arquitectónico
Reproductor de audio simple para notificaciones del sistema (ej. batería baja, sonidos de inicio). Centraliza el manejo del hardware de audio, permitiendo a otras capas reproducir alertas predefinidas.

## Entorno y Dependencias
Depende de la API I2S de ESP-IDF y del driver del codec de audio (ES8311, instanciado en componentes adicionales). Requiere la configuración de red y logs para inicializar correctamente.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Bus I2S para enviar tramas de audio digital al codec. I2C (implícito) para configuración de registros del codec de sonido.
- **Software:** API C asíncrona/bloqueante según uso (`audio_player_play()` con parámetros enumerados `BATTERY_LOW`, `STARTUP`), y control de volumen.

## Flujo de Ejecución Lógico
Se inicializa una vez al arrancar (`audio_player_init()`). Proveedor de funciones por demanda que transmiten un buffer de audio precargado o flash directo hacia el bus I2S cada vez que ocurre un evento del sistema.

## Funciones Principales y Parámetros
- `audio_player_init(void)`: Inicializa el reproductor de audio (buses I2S y Codec). No recibe parámetros. Devuelve `ESP_OK` si tuvo éxito.
- `audio_player_play(audio_sound_t sound)`: Reproduce un sonido predefinido a volumen normal.
  - `sound`: Enum del sonido a reproducir (`BATTERY_LOW`, `STARTUP`, etc.).
- `audio_player_play_vol(audio_sound_t sound, uint8_t volume)`: Reproduce un sonido con un volumen específico.
  - `sound`: Enum del sonido.
  - `volume`: Nivel de volumen deseado (0-100).
- `audio_player_stop(void)`: Detiene la reproducción actual de audio. No requiere parámetros.

## Puntos Críticos y Depuración
- **I2S Under-run:** Si el sistema general se satura o la tarea tiene baja prioridad, el bus I2S puede quedarse sin datos generando ruidos o cortes.
- **Buses bloqueados:** Problemas I2C durante la inicialización del codec (Pull-ups faltantes) que podrían fallar la inicialización en frío.
