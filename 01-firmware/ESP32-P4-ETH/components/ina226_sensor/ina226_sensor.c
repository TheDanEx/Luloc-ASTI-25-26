/**
 * @file ina226_sensor.c
 * @brief INA226 Driver implementation for Lurloc-ASTI Robot.
 * 
 * Complies with strict project standards: English naming, snake_case,
 * defensive programming, and minimal comments.
 */

#include "ina226_sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <stdlib.h>

static const char *TAG = "INA226";

/* Register Addresses */
#define REG_CONFIG           0x00
#define REG_SHUNT_VOLT       0x01
#define REG_BUS_VOLT         0x02
#define REG_POWER            0x03
#define REG_CURRENT          0x04
#define REG_CALIBRATION      0x05
#define REG_MASK_ENABLE      0x06
#define REG_ALERT_LIMIT      0x07
#define REG_MANUFACTURER_ID  0xFE
#define REG_DIE_ID           0xFF

/* Constant IDs */
#define ID_MANUFACTURER      0x5449  // "TI"
#define ID_DIE               0x2260

/* Configuration Bits */
#define CFG_RESET            BIT(15)
#define CFG_AVG_16           (0x06 << 9)
#define CFG_VBUS_1MS         (0x04 << 6)
#define CFG_VSH_1MS          (0x04 << 3)
#define CFG_MODE_CONT        (0x07)

/* Physical Constants */
#define VBUS_LSB_MV          1.25f
#define I2C_TIMEOUT_MS       100
#define DEFAULT_I2C_ADDR     0x40

static bool  s_initialized = false;
static int   s_i2c_port    = 0;
static float s_current_lsb = 0.0f;
static float s_power_lsb   = 0.0f;

/**
 * @brief Thread-safe register write helper.
 */
static esp_err_t write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_write_to_device(s_i2c_port, DEFAULT_I2C_ADDR, buf, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * @brief Thread-safe register read helper.
 */
static esp_err_t read_reg(uint8_t reg, uint16_t *value)
{
    if (!value) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = {0};
    esp_err_t ret = i2c_master_write_read_device(s_i2c_port, DEFAULT_I2C_ADDR, &reg, 1, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret == ESP_OK) {
        *value = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}

/**
 * @brief Internal calibration routine using INA226 equations.
 */
static esp_err_t run_calibration(void)
{
    float shunt_ohms = 0.01f;
#ifdef CONFIG_INA226_SHUNT_OHMS
    shunt_ohms = (float)atof(CONFIG_INA226_SHUNT_OHMS);
#endif

    float max_amps = 10.0f;
#ifdef CONFIG_INA226_MAX_EXPECTED_AMPS
    max_amps = (float)atof(CONFIG_INA226_MAX_EXPECTED_AMPS);
#endif

    /* Equation 2: Current_LSB = Max_Expected_I / 32768 */
    s_current_lsb = max_amps / 32768.0f;
    
    /* Equation 1: CAL = 0.00512 / (Current_LSB * Rshunt) */
    uint16_t cal_val = (uint16_t)(0.00512f / (s_current_lsb * shunt_ohms));
    
    /* Equation 3: Power_LSB = 25 * Current_LSB */
    s_power_lsb = 25.0f * s_current_lsb;

    ESP_LOGI(TAG, "Syncing scale: Rshunt=%.3f, MaxA=%.1f, Cal=%u", shunt_ohms, max_amps, cal_val);

    ESP_ERROR_CHECK(write_reg(REG_CONFIG, CFG_RESET));
    vTaskDelay(pdMS_TO_TICKS(2));

    uint16_t config = CFG_AVG_16 | CFG_VBUS_1MS | CFG_VSH_1MS | CFG_MODE_CONT;
    ESP_ERROR_CHECK(write_reg(REG_CONFIG, config));
    ESP_ERROR_CHECK(write_reg(REG_CALIBRATION, cal_val));

    return ESP_OK;
}

esp_err_t ina226_sensor_init(void)
{
    if (s_initialized) return ESP_OK;

#ifdef CONFIG_INA226_I2C_PORT
    s_i2c_port = CONFIG_INA226_I2C_PORT;
#endif

    uint16_t mfr_id = 0, die_id = 0;
    if (read_reg(REG_MANUFACTURER_ID, &mfr_id) != ESP_OK || mfr_id != ID_MANUFACTURER) {
        ESP_LOGE(TAG, "Verification failed (MFR_ID=0x%04X). Is the bus ready?", mfr_id);
        return ESP_FAIL;
    }
    read_reg(REG_DIE_ID, &die_id);
    ESP_LOGI(TAG, "Detected INA226 (DieID=0x%04X) on Port %d", die_id, s_i2c_port);

    s_initialized = true;
    return run_calibration();
}

esp_err_t ina226_sensor_read_bus_voltage_mv(float *voltage_mv)
{
    if (!voltage_mv || !s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t val = 0;
    esp_err_t err = read_reg(REG_BUS_VOLT, &val);
    if (err == ESP_OK) *voltage_mv = (float)val * VBUS_LSB_MV;
    return err;
}

esp_err_t ina226_sensor_read_current_ma(float *current_ma)
{
    if (!current_ma || !s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t val = 0;
    esp_err_t err = read_reg(REG_CURRENT, &val);
    if (err == ESP_OK) *current_ma = (float)((int16_t)val) * s_current_lsb * 1000.0f;
    return err;
}

esp_err_t ina226_sensor_read_power_mw(float *power_mw)
{
    if (!power_mw || !s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t val = 0;
    esp_err_t err = read_reg(REG_POWER, &val);
    if (err == ESP_OK) *power_mw = (float)val * s_power_lsb * 1000.0f;
    return err;
}
