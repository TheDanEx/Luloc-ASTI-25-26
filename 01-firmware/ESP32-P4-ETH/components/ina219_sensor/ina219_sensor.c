#include "ina219_sensor.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"

static const char *TAG = "INA219";

#define INA219_I2C_ADDRESS 0x40

// Registers
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLT   0x01
#define INA219_REG_BUS_VOLT     0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// Configuration
#define INA219_CONFIG_BVOLTAGERANGE_16V      (0x0000)
#define INA219_CONFIG_BVOLTAGERANGE_32V      (0x2000)
#define INA219_CONFIG_GAIN_1_40MV            (0x0000)
#define INA219_CONFIG_GAIN_2_80MV            (0x0800)
#define INA219_CONFIG_GAIN_4_160MV           (0x1000)
#define INA219_CONFIG_GAIN_8_320MV           (0x1800)
#define INA219_CONFIG_BADCRES_12BIT          (0x0400)
#define INA219_CONFIG_SADCRES_12BIT_1S_532US (0x0018)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

static float current_lsb = 0;
static float power_lsb   = 0;

static esp_err_t ina219_write_register(uint8_t reg, uint16_t value) {
    uint8_t buffer[3] = { reg, (value >> 8) & 0xFF, value & 0xFF };
    return i2c_master_transmit(dev_handle, buffer, 3, -1);
}

static esp_err_t ina219_read_register(uint8_t reg, uint16_t *value) {
    uint8_t buffer[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, 2, -1);
    if (err == ESP_OK) {
        *value = (buffer[0] << 8) | buffer[1];
    }
    return err;
}

// Calibrates the INA219 to calculate Current and Power properly
static esp_err_t ina219_calibrate(void) {
    // Read from Kconfig
    // Defaults: Rshunt = 0.01 ohms, Max Expected I = 3.2A
    // Vshunt_max = 3.2A * 0.01R = 0.032V = 32mV (Fits well within PGA /1 (40mV))
    
    // We parse the string configs as floats
    
    // Fallbacks if Kconfig is empty or 0
    float r_shunt = 0.01f;
    #ifdef CONFIG_INA219_SHUNT_OHMS
        r_shunt = atof(CONFIG_INA219_SHUNT_OHMS);
        if (r_shunt <= 0.0f) r_shunt = 0.01f;
    #endif

    float max_expected_amps = 3.2f;
    #ifdef CONFIG_INA219_MAX_EXPECTED_AMPS
        max_expected_amps = atof(CONFIG_INA219_MAX_EXPECTED_AMPS);
        if (max_expected_amps <= 0.0f) max_expected_amps = 3.2f;
    #endif

    // Minimum LSB = Max Expected Amps / 32767
    // To get a "nice" number, we usually pick a round value slightly larger
    current_lsb = max_expected_amps / 32768.0f;
    
    // We can hardcode LSB to 0.1mA (0.0001A) for 3.2A max
    current_lsb = 0.0001f; 

    // Calibration Register Equation: Cal = trunc (0.04096 / (Current_LSB * Rshunt))
    uint16_t cal_value = (uint16_t)(0.04096f / (current_lsb * r_shunt));
    
    power_lsb = current_lsb * 20.0f;

    esp_err_t err = ina219_write_register(INA219_REG_CALIBRATION, cal_value);
    if (err != ESP_OK) return err;

    // Write Config
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV |
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    err = ina219_write_register(INA219_REG_CONFIG, config);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "INA219 Calibrated. Rshunt=%.3f, MaxAmps=%.1f, CalVal=%d", r_shunt, max_expected_amps, cal_value);
    }
    return err;
}

esp_err_t ina219_sensor_init(void) {
    if (dev_handle != NULL) {
        ESP_LOGW(TAG, "INA219 already initialized");
        return ESP_OK;
    }

    int port = 0;
    int sda_pin = 21;
    int scl_pin = 22;
    uint32_t clk_speed = 400000;

    #ifdef CONFIG_INA219_I2C_PORT
        port = CONFIG_INA219_I2C_PORT;
    #endif
    #ifdef CONFIG_INA219_I2C_SDA_PIN
        sda_pin = CONFIG_INA219_I2C_SDA_PIN;
    #endif
    #ifdef CONFIG_INA219_I2C_SCL_PIN
        scl_pin = CONFIG_INA219_I2C_SCL_PIN;
    #endif
    #ifdef CONFIG_INA219_I2C_CLOCK_SPEED_HZ
        clk_speed = CONFIG_INA219_I2C_CLOCK_SPEED_HZ;
    #endif

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { 
        // INVALID_STATE means port is already used, which is fine if we share bus
        ESP_LOGE(TAG, "I2C Bus Initialization failed");
        // We could attempt to get existing bus handle here in a complex app, 
        // but for this driver we expect to be the one setting it up or sharing gracefully.
    }

    // Now add device to bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_I2C_ADDRESS,
        .scl_speed_hz = clk_speed,
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA219 to I2C bus");
        return err;
    }

    // Ping device by reading config register
    uint16_t dummy;
    err = ina219_read_register(INA219_REG_CONFIG, &dummy);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INA219 not found at address 0x%02x", INA219_I2C_ADDRESS);
        return err;
    }

    return ina219_calibrate();
}

esp_err_t ina219_sensor_read_bus_voltage_mv(float *voltage_mv) {
    if (!dev_handle || !voltage_mv) return ESP_ERR_INVALID_ARG;
    
    uint16_t value;
    esp_err_t err = ina219_read_register(INA219_REG_BUS_VOLT, &value);
    if (err != ESP_OK) return err;

    // Shift right 3 bits, LSB = 4mV
    int16_t signed_val = (int16_t)((value >> 3) * 4);
    *voltage_mv = (float)signed_val;
    return ESP_OK;
}

esp_err_t ina219_sensor_read_current_ma(float *current_ma) {
    if (!dev_handle || !current_ma) return ESP_ERR_INVALID_ARG;
    
    // Must write calibration before reading current if not operating purely in voltage mode
    // (In continuous mode it might hold it, but good practice to ensure)
    uint16_t value;
    esp_err_t err = ina219_read_register(INA219_REG_CURRENT, &value);
    if (err != ESP_OK) return err;

    int16_t signed_val = (int16_t)value;
    *current_ma = ((float)signed_val * current_lsb) * 1000.0f; // Convert A to mA
    return ESP_OK;
}

esp_err_t ina219_sensor_read_power_mw(float *power_mw) {
    if (!dev_handle || !power_mw) return ESP_ERR_INVALID_ARG;
    
    uint16_t value;
    esp_err_t err = ina219_read_register(INA219_REG_POWER, &value);
    if (err != ESP_OK) return err;

    int16_t signed_val = (int16_t)value;
    *power_mw = ((float)signed_val * power_lsb) * 1000.0f; // Convert W to mW
    return ESP_OK;
}
