/**
 * @file audio_player.h
 * @brief Simple Audio Player Component
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Available sounds to play
 */
typedef enum {
    BATTERY_LOW,
    STARTUP,
    SOUND_MAX
} audio_sound_t;

/**
 * @brief Initialize the audio player (I2S + Codec)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t audio_player_init(void);

/**
 * @brief Play a specific sound
 * 
 * @param sound The sound enum to play
 * @return esp_err_t ESP_OK on success
 */
esp_err_t audio_player_play(audio_sound_t sound);

/**
 * @brief Play a specific sound with volume override
 * 
 * @param sound The sound enum to play
 * @param volume Volume (0-100)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t audio_player_play_vol(audio_sound_t sound, uint8_t volume);

/**
 * @brief Stop current playback
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t audio_player_stop(void);

#ifdef __cplusplus
}
#endif
