#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

// Ignoramos el aviso de librería obsoleta para que no detenga el "Build"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "driver/adc.h"

// --- CONFIGURACIÓN DE USUARIO ---
bool MODO_CALIBRACION = false; // Cambia a 'false' para usar los valores guardados
#define NUM_IR 8
#define PIN_ENABLE 33          // Pin que activa la regleta de sensores

// ATENCIÓN: En la API antigua del ADC, no se leen los GPIO directamente, 
// sino su "Canal ADC1" correspondiente. 
// Mapea aquí los 8 canales exactos que coincidan con tus pines físicos en la P4.
// (Esto es un ejemplo secuencial, ajusta según el pinout de tu placa)
const adc1_channel_t canales_adc[NUM_IR] = {
    ADC1_CHANNEL_0, 
    ADC1_CHANNEL_1, 
    ADC1_CHANNEL_2, 
    ADC1_CHANNEL_3, 
    ADC1_CHANNEL_4, 
    ADC1_CHANNEL_5, 
    ADC1_CHANNEL_6, 
    ADC1_CHANNEL_7
};

// Variables globales (Ajustadas a int32_t para cumplir con el estándar NVS)
int32_t umbrales[NUM_IR];      
int32_t negro_calib[NUM_IR];   // Guardamos los valores de negro para el filtro de aire
int ir_values[NUM_IR];
bool binary[NUM_IR];

// --- PROTOTIPOS ---
void configurar_adc_bloque1(void);
int leer_sensor_adc(int indice);
void ejecutarCalibracion(void);
void cargarCalibracion(void);

// --- CONFIGURACIÓN DEL ADC ---
void configurar_adc_bloque1(void) {
    printf("Configurando ADC1 (Legacy API)...\n");
    // 1. Resolución a 12 bits (nos dará valores de 0 a 4095)
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // 2. Configurar la atenuación a 11dB para cada canal (Rango 0V - 3.3V)
    for (int i = 0; i < NUM_IR; i++) {
        adc1_config_channel_atten(canales_adc[i], ADC_ATTEN_DB_11);
    }
}

int leer_sensor_adc(int indice) {
    // Extrae el valor analógico real (0-4095) del canal configurado
    return adc1_get_raw(canales_adc[indice]);
}

// --- LÓGICA DE CALIBRACIÓN Y MEMORIA ---
void ejecutarCalibracion(void) {
    int32_t blanco[NUM_IR];
    int32_t negro[NUM_IR];

    printf("\n>>> MODO CALIBRACIÓN ACTIVO\n");
    printf("Poner sobre BLANCO. Lectura en 4 seg...\n");
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    for (int i = 0; i < NUM_IR; i++) {
        int32_t suma = 0;
        for (int x = 0; x < 50; x++) suma += leer_sensor_adc(i);
        blanco[i] = suma / 50;
        printf("S%d Blanco: %ld\n", i, blanco[i]);
    }

    printf("\nPoner sobre NEGRO (Línea). Lectura en 4 seg...\n");
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    for (int i = 0; i < NUM_IR; i++) {
        int32_t suma = 0;
        for (int x = 0; x < 50; x++) suma += leer_sensor_adc(i);
        negro[i] = suma / 50;
        negro_calib[i] = negro[i]; // Guardamos para el filtro de aire
        printf("S%d Negro: %ld\n", i, negro[i]);
    }

    printf("\nGuardando datos en memoria...\n");
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    
    for (int i = 0; i < NUM_IR; i++) {
        umbrales[i] = (blanco[i] + negro[i]) / 2;
        
        char key_u[4], key_n[4];
        sprintf(key_u, "u%d", i);
        sprintf(key_n, "n%d", i);
        
        nvs_set_i32(my_handle, key_u, umbrales[i]);
        nvs_set_i32(my_handle, key_n, negro_calib[i]);
        
        printf("S%d Umbral: %ld | Negro: %ld\n", i, umbrales[i], negro_calib[i]);
    }
    nvs_commit(my_handle);
    nvs_close(my_handle);
    printf("¡CALIBRACIÓN LISTA! Cambia MODO_CALIBRACION a false y resube.\n");
}

void cargarCalibracion(void) {
    printf("\n>>> MODO NORMAL: Cargando memoria...\n");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    
    if (err != ESP_OK) {
        printf("Error abriendo NVS, usando valores por defecto.\n");
        for (int i = 0; i < NUM_IR; i++) {
            umbrales[i] = 2000;
            negro_calib[i] = 3500;
        }
    } else {
        for (int i = 0; i < NUM_IR; i++) {
            char key_u[4], key_n[4];
            sprintf(key_u, "u%d", i);
            sprintf(key_n, "n%d", i);
            
            nvs_get_i32(my_handle, key_u, &umbrales[i]);
            nvs_get_i32(my_handle, key_n, &negro_calib[i]);
            printf("S%d: U=%ld N=%ld | ", i, umbrales[i], negro_calib[i]);
        }
        nvs_close(my_handle);
    }
    printf("\nListo para correr.\n");
}

// --- EQUIVALENTE A SETUP() Y LOOP() ---
void app_main(void) {
    // 1. Inicializar Memoria Flash (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. Configurar Pin de Activación de Sensores
    gpio_reset_pin(PIN_ENABLE);
    gpio_set_direction(PIN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_ENABLE, 1);
    
    // 3. Inicializar y configurar el bloque ADC1
    configurar_adc_bloque1();

    // 4. Setup (Calibrar o Cargar memoria)
    if (MODO_CALIBRACION) {
        ejecutarCalibracion();
    } else {
        cargarCalibracion();
    }

    // 5. Loop Infinito
    while (1) {
        for (int i = 0; i < NUM_IR; i++) {
            ir_values[i] = leer_sensor_adc(i);
            
            // --- LÓGICA CON MARGEN DE SEGURIDAD (ANTI-BACHES) ---
            int32_t limiteAire = negro_calib[i] * 1.15; 
            if (limiteAire > 4050) limiteAire = 4050; 

            int32_t umbralConMargen = umbrales[i] + 250;

            if (ir_values[i] > limiteAire) {
                binary[i] = 0; // Demasiado alto/aire -> Blanco
            } 
            else if (ir_values[i] > umbralConMargen) {
                binary[i] = 1; // Negro confirmado
            } 
            else {
                binary[i] = 0; // Blanco/Suelo normal
            }
        }

        // Visualización gráfica
        printf("Binario: ");
        for (int i = 0; i < NUM_IR; i++) {
            printf(binary[i] ? "0 " : "1 "); 
        }
        // Mostramos el valor crudo del primer sensor (como ejemplo) para que veas que no lee solo 0 o 4095
        printf(" | S0 Raw: %d\r", ir_values[0]); 
        fflush(stdout);
        
        vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
}