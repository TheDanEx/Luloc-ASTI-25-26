#include "audio_player.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_check.h"
#include "es8311.h"

// --- Configuration (Hardcoded for ESP32-P4 per example) ---
#define I2C_NUM         (0)
#define I2C_SCL_IO      (GPIO_NUM_8)
#define I2C_SDA_IO      (GPIO_NUM_7)
#define GPIO_OUTPUT_PA  (GPIO_NUM_53)

#define I2S_NUM         (0)
#define I2S_MCK_IO      (GPIO_NUM_13)
#define I2S_BCK_IO      (GPIO_NUM_12)
#define I2S_WS_IO       (GPIO_NUM_10)
#define I2S_DO_IO       (GPIO_NUM_9)
#define I2S_DI_IO       (GPIO_NUM_11)

#define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_MCLK_MULTIPLE   (384)
#define EXAMPLE_MCLK_FREQ_HZ    (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)
#define EXAMPLE_VOICE_VOLUME    (60) // Default volume

static const char *TAG = "aud_play";

static i2s_chan_handle_t tx_handle = NULL;
static TaskHandle_t play_task_handle = NULL;
static es8311_handle_t es_handle = NULL;

// --- Embedded Sounds ---
// Note: CMake EMBED_FILES "sounds/battery_low.pcm" -> _binary_battery_low_pcm_start (usually flattens path)
// If link error occurs, check the actual symbol name.
extern const uint8_t battery_low_pcm_start[] asm("_binary_battery_low_pcm_start");
extern const uint8_t battery_low_pcm_end[]   asm("_binary_battery_low_pcm_end");

extern const uint8_t startup_pcm_start[] asm("_binary_startup_pcm_start");
extern const uint8_t startup_pcm_end[]   asm("_binary_startup_pcm_end");

typedef struct {
    const uint8_t *start;
    const uint8_t *end;
    uint8_t volume;
} sound_asset_t;

static sound_asset_t get_sound_asset(audio_sound_t sound) {
    sound_asset_t asset = {NULL, NULL, 60}; // Default volume 60
    switch (sound) {
        case BATTERY_LOW:
            asset.start = battery_low_pcm_start;
            asset.end = battery_low_pcm_end;
            asset.volume = 70;
            break;
        case STARTUP:
            asset.start = startup_pcm_start;
            asset.end = startup_pcm_end;
            asset.volume = 70;
            break;
        default:
            break;
    }
    return asset;
}

// --- Internal Functions ---

static void gpio_pa_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PA),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_PA, 1); // Enable PA
}

static esp_err_t i2c_init(void)
{
    const i2c_config_t es_i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    return i2c_param_config(I2C_NUM, &es_i2c_cfg) ?: i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t codec_init(void)
{
    es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
    if (!es_handle) {
        return ESP_FAIL;
    }
    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
        .sample_frequency = EXAMPLE_SAMPLE_RATE
    };

    ESP_RETURN_ON_ERROR(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16), TAG, "es8311 init failed");
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE, EXAMPLE_SAMPLE_RATE), TAG, "set sample freq failed");
    ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set volume failed");
    ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "mic config failed");
    
    return ESP_OK;
}

static esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 8;     // Increase from default (6)
    chan_cfg.dma_frame_num = 1024; // Increase from default (240) to ~1KB per desc
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx_handle, NULL), TAG, "i2s new channel failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_handle, &std_cfg), TAG, "i2s init std failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx_handle), TAG, "i2s enable failed");
    return ESP_OK;
}

static void play_task(void *args)
{
    sound_asset_t *asset = (sound_asset_t *)args;
    size_t bytes_written = 0;
    
    // Preload
    // i2s_channel_preload_data(tx_handle, asset->start, asset->end - asset->start, &bytes_written);
    
    // Write
    i2s_channel_write(tx_handle, asset->start, asset->end - asset->start, &bytes_written, portMAX_DELAY);
    
    if (bytes_written > 0) {
        ESP_LOGD(TAG, "Played %d bytes", bytes_written);
    } // else log error

    free(asset); // Free the malloced struct
    play_task_handle = NULL;
    vTaskDelete(NULL);
}

// --- Public API ---

esp_err_t audio_player_init(void)
{
    static bool initialized = false;
    if (initialized) return ESP_OK;

    gpio_pa_init();
    
    ESP_RETURN_ON_ERROR(i2c_init(), TAG, "I2C Init Failed");
    ESP_RETURN_ON_ERROR(codec_init(), TAG, "Codec Init Failed");
    ESP_RETURN_ON_ERROR(i2s_init(), TAG, "I2S Init Failed");

    initialized = true;
    ESP_LOGD(TAG, "Audio Player Initialized");
    return ESP_OK;
}

esp_err_t audio_player_play(audio_sound_t sound)
{
    sound_asset_t asset = get_sound_asset(sound);
    return audio_player_play_vol(sound, asset.volume);
}

esp_err_t audio_player_play_vol(audio_sound_t sound, uint8_t volume)
{
    if (play_task_handle != NULL) {
        // Stop current? Or ignore?
        // Simple strategy: wait for previous to finish or just ignore if one is playing.
        // For better responsiveness, we might want to kill the previous task, but that's risky with I2S handles.
        // Let's just log busy for now or queue it. 
        // For "Easy to use", maybe we just ignore specific overlapping requests for now.
        ESP_LOGW(TAG, "Audio player busy");
        return ESP_ERR_INVALID_STATE; 
    }

    sound_asset_t asset = get_sound_asset(sound);
    if (asset.start == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    sound_asset_t *task_arg = malloc(sizeof(sound_asset_t));
    *task_arg = asset;

    // Set Volume for this specific sound
    if (es_handle) {
        es8311_voice_volume_set(es_handle, volume, NULL);
    }

    xTaskCreate(play_task, "audio_play", 4096, task_arg, 5, &play_task_handle);
    return ESP_OK;
}

esp_err_t audio_player_stop(void)
{
    // Not implemented fully yet (needs safe task deletion)
    return ESP_ERR_NOT_SUPPORTED;
}
