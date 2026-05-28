idf_component_register(
    SRCS "src/audio_player.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2s esp_driver_gpio espressif__es8311 driver
    EMBED_FILES "sounds/battery_low.pcm" 
    "sounds/startup.pcm"
    "sounds/in_the_end.pcm"
    "sounds/tokyo_drift.pcm"
    "sounds/hola.pcm"
    "sounds/demacia.pcm"
    "sounds/round_1_fight.pcm"
    "sounds/round_2_fight.pcm"
    "sounds/round_3_fight.pcm"
)
