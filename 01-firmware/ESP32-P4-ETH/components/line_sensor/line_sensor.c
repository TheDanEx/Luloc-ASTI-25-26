#include "line_sensor.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LINE_SENSOR";

// Static instance of configuration and calibration
static line_sensor_config_t s_sensors[LINE_SENSOR_COUNT];
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

// ADC Channel handles for oneshot mode
static int s_adc_channels[LINE_SENSOR_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1};

esp_err_t line_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing line sensor array");

    // 1. Setup sensor mapping from Kconfig
    // Digital Pins
    s_sensors[0] = (line_sensor_config_t){.type = LINE_SENSOR_DIGITAL, .pin = CONFIG_LINE_SENSOR_PIN_D1, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D1};
    s_sensors[1] = (line_sensor_config_t){.type = LINE_SENSOR_DIGITAL, .pin = CONFIG_LINE_SENSOR_PIN_D2, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D2};
    s_sensors[6] = (line_sensor_config_t){.type = LINE_SENSOR_DIGITAL, .pin = CONFIG_LINE_SENSOR_PIN_D7, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D7};
    s_sensors[7] = (line_sensor_config_t){.type = LINE_SENSOR_DIGITAL, .pin = CONFIG_LINE_SENSOR_PIN_D8, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D8};

    // 1.1 Enable IR Emmitters Pin
    gpio_config_t ir_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_LINE_SENSOR_PIN_IR_EN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&ir_conf);
    gpio_set_level(CONFIG_LINE_SENSOR_PIN_IR_EN, 1); // Power ON
    ESP_LOGI(TAG, "IR emitters enabled on GPIO %d", CONFIG_LINE_SENSOR_PIN_IR_EN);

    // Analog Pins
    s_sensors[2] = (line_sensor_config_t){.type = LINE_SENSOR_ANALOG, .pin = CONFIG_LINE_SENSOR_PIN_D3, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D3, .min_value = CONFIG_LINE_SENSOR_DEFAULT_MIN, .max_value = CONFIG_LINE_SENSOR_DEFAULT_MAX};
    s_sensors[3] = (line_sensor_config_t){.type = LINE_SENSOR_ANALOG, .pin = CONFIG_LINE_SENSOR_PIN_D4, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D4, .min_value = CONFIG_LINE_SENSOR_DEFAULT_MIN, .max_value = CONFIG_LINE_SENSOR_DEFAULT_MAX};
    s_sensors[4] = (line_sensor_config_t){.type = LINE_SENSOR_ANALOG, .pin = CONFIG_LINE_SENSOR_PIN_D5, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D5, .min_value = CONFIG_LINE_SENSOR_DEFAULT_MIN, .max_value = CONFIG_LINE_SENSOR_DEFAULT_MAX};
    s_sensors[5] = (line_sensor_config_t){.type = LINE_SENSOR_ANALOG, .pin = CONFIG_LINE_SENSOR_PIN_D6, .weight_mm = (float)CONFIG_LINE_SENSOR_WEIGHT_D6, .min_value = CONFIG_LINE_SENSOR_DEFAULT_MIN, .max_value = CONFIG_LINE_SENSOR_DEFAULT_MAX};

    s_adc_channels[2] = CONFIG_LINE_SENSOR_CHAN_D3;
    s_adc_channels[3] = CONFIG_LINE_SENSOR_CHAN_D4;
    s_adc_channels[4] = CONFIG_LINE_SENSOR_CHAN_D5;
    s_adc_channels[5] = CONFIG_LINE_SENSOR_CHAN_D6;

    // 2. Configure GPIOs for digital pins
    uint64_t pin_mask = 0;
    for(int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if(s_sensors[i].type == LINE_SENSOR_DIGITAL) {
            pin_mask |= (1ULL << s_sensors[i].pin);
        }
    }

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = pin_mask,
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    // 3. Configure ADC for analog pins
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1, // Defaulting to unit 1 for simplicity, can be Kconfig
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // High voltage range (up to 3.3V roughly on P4)
    };

    for(int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if(s_sensors[i].type == LINE_SENSOR_ANALOG) {
            ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_adc_channels[i], &chan_config));
        }
    }

    return ESP_OK;
}

void line_sensor_deinit(void)
{
    if(s_adc_handle) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
}

void line_sensor_read_raw(uint32_t *raw_values, uint32_t num_samples)
{
    if(!raw_values) return;

    for(int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if(s_sensors[i].type == LINE_SENSOR_DIGITAL) {
            raw_values[i] = gpio_get_level(s_sensors[i].pin);
        } else if(s_sensors[i].type == LINE_SENSOR_ANALOG) {
            uint32_t sum = 0;
            for(int s = 0; s < num_samples; s++) {
                int val;
                adc_oneshot_read(s_adc_handle, s_adc_channels[i], &val);
                sum += val;
            }
            raw_values[i] = sum / num_samples;
        }
    }
}

void line_sensor_read_norm(float *norm_values, const uint32_t *raw_values_in, uint32_t num_samples)
{
    if(!norm_values) return;
    
    uint32_t raw_buf[LINE_SENSOR_COUNT];
    const uint32_t *raw = raw_values_in;

    if(!raw) {
        line_sensor_read_raw(raw_buf, num_samples);
        raw = raw_buf;
    }

    for(int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if(s_sensors[i].type == LINE_SENSOR_DIGITAL) {
            norm_values[i] = (float)raw[i];
        } else {
            // Analog normalization: (val - min) / (max - min) constrained to [0,1]
            uint32_t min = s_sensors[i].min_value;
            uint32_t max = s_sensors[i].max_value;
            
            if(raw[i] <= min) norm_values[i] = 0.0f;
            else if(raw[i] >= max) norm_values[i] = 1.0f;
            else norm_values[i] = (float)(raw[i] - min) / (float)(max - min);
        }
    }
}

void line_sensor_read_bin(uint8_t *bin_values, const float *norm_values_in, uint32_t num_samples)
{
    if(!bin_values) return;

    float norm_buf[LINE_SENSOR_COUNT];
    const float *norm = norm_values_in;

    if(!norm) {
        line_sensor_read_norm(norm_buf, NULL, num_samples);
        norm = norm_buf;
    }

    for(int i = 0; i < LINE_SENSOR_COUNT; i++) {
        bin_values[i] = (norm[i] > 0.5f) ? 1 : 0;
    }
}

float line_sensor_read_line_position(const float *norm_values_in, uint32_t num_samples)
{
    float norm_buf[LINE_SENSOR_COUNT];
    const float *norm = norm_values_in;

    if (!norm) {
        line_sensor_read_norm(norm_buf, NULL, num_samples);
        norm = norm_buf;
    }

    float weighted_sum = 0.0f;
    float total_norm = 0.0f;

    for(int i = 0; i < LINE_SENSOR_COUNT; i++) {
        weighted_sum += norm[i] * s_sensors[i].weight_mm;
        total_norm += norm[i];
    }

    if(total_norm < 0.001f) {
        return 0.0f; // No line detected (center) or handle externally
    }

    return weighted_sum / total_norm;
}

void line_sensor_set_calibration(uint8_t index, uint32_t min_val, uint32_t max_val)
{
    if(index < LINE_SENSOR_COUNT) {
        s_sensors[index].min_value = min_val;
        s_sensors[index].max_value = max_val;
    }
}

void line_sensor_get_calibration(uint8_t index, uint32_t *min_val, uint32_t *max_val)
{
    if(index < LINE_SENSOR_COUNT) {
        if(min_val) *min_val = s_sensors[index].min_value;
        if(max_val) *max_val = s_sensors[index].max_value;
    }
}
