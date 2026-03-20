/**
 * @file ina226_sensor.c
 * @brief INA226 Driver - Prístine Standard for Lurloc-ASTI.
 */

#include "ina226_sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "audio_player.h"
#include "esp_timer.h"
#include <stdlib.h>

// =============================================================================
// Constants & Config
// =============================================================================
static const char *TAG = "INA226";

#define REG_CONFIG           0x00
#define REG_BUS_VOLT         0x02
#define REG_POWER            0x03
#define REG_CURRENT          0x04
#define REG_CALIBRATION      0x05
#define REG_MANUFACTURER_ID  0xFE

#define CFG_RESET            BIT(15)
#define CFG_AVG_16           (0x06 << 9)
#define CFG_VBUS_1MS         (0x04 << 6)
#define CFG_VSH_1MS          (0x04 << 3)
#define CFG_MODE_CONT        (0x07)

#define VBUS_LSB_MV          1.25f
#define I2C_TIMEOUT_MS       100
#define DEFAULT_I2C_ADDR     0x40

#if defined(CONFIG_INA226_LOW_VOLTAGE_SOUND_STARTUP)
    #define LOW_VOLTAGE_SOUND STARTUP
#else
    #define LOW_VOLTAGE_SOUND BATTERY_LOW
#endif

#if defined(CONFIG_INA226_HIGH_CURRENT_SOUND_STARTUP)
    #define HIGH_CURRENT_SOUND STARTUP
#else
    #define HIGH_CURRENT_SOUND BATTERY_LOW
#endif

#define ALERT_COOLDOWN_SEC   30

// =============================================================================
// Static Variables
// =============================================================================
static bool  s_initialized  = false;
static int   s_i2c_port     = 0;
static float s_current_lsb  = 0.0f;
static float s_power_lsb    = 0.0f;

// =============================================================================
// Internal Helpers
// =============================================================================

/**
 * Write a 16-bit word to a device register via I2C.
 * Uses big-endian byte order as required by INA226.
 */
static esp_err_t write_reg(uint8_t reg, uint16_t value) {
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_write_to_device(s_i2c_port, DEFAULT_I2C_ADDR, buf, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * Read a 16-bit word from a device register via I2C.
 */
static esp_err_t read_reg(uint8_t reg, uint16_t *value) {
    if (!value) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = {0};
    esp_err_t ret = i2c_master_write_read_device(s_i2c_port, DEFAULT_I2C_ADDR, &reg, 1, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret == ESP_OK) *value = ((uint16_t)buf[0] << 8) | buf[1];
    return ret;
}

// =============================================================================
// Public API: Lifecycle
// =============================================================================

/**
 * Initialize the INA226 power monitor.
 * Configures shunt resistance, expected current range, and sampling averaging.
 */
esp_err_t ina_init(void) {
    if (s_initialized) return ESP_OK;

#ifdef CONFIG_INA226_I2C_PORT
    s_i2c_port = CONFIG_INA226_I2C_PORT;
#endif

    uint16_t mfr_id = 0;
    if (read_reg(REG_MANUFACTURER_ID, &mfr_id) != ESP_OK || mfr_id != 0x5449) return ESP_FAIL;

    float shunt_ohms = 0.01f, max_expected_amps = 10.0f;
#ifdef CONFIG_INA226_SHUNT_OHMS
    shunt_ohms = (float)atof(CONFIG_INA226_SHUNT_OHMS);
#endif
#ifdef CONFIG_INA226_MAX_EXPECTED_AMPS
    max_expected_amps = (float)atof(CONFIG_INA226_MAX_EXPECTED_AMPS);
#endif

    s_current_lsb = max_expected_amps / 32768.0f;
    uint16_t cal_val = (uint16_t)(0.00512f / (s_current_lsb * shunt_ohms));
    s_power_lsb = 25.0f * s_current_lsb;

    write_reg(REG_CONFIG, CFG_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    write_reg(REG_CONFIG, CFG_AVG_16 | CFG_VBUS_1MS | CFG_VSH_1MS | CFG_MODE_CONT);
    write_reg(REG_CALIBRATION, cal_val);

    s_initialized = true;
    return ESP_OK;
}

// =============================================================================
// Public API: Metrics
// =============================================================================

esp_err_t ina_read_voltage(float *voltage_mv) {
    if (!voltage_mv || !s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t raw_val;
    esp_err_t ret = read_reg(REG_BUS_VOLT, &raw_val);
    if (ret == ESP_OK) *voltage_mv = (float)raw_val * VBUS_LSB_MV;
    return ret;
}

esp_err_t ina_read_current(float *current_ma) {
    if (!current_ma || !s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t raw_val;
    esp_err_t ret = read_reg(REG_CURRENT, &raw_val);
    if (ret == ESP_OK) *current_ma = (float)((int16_t)raw_val) * s_current_lsb * 1000.0f;
    return ret;
}

esp_err_t ina_read_power(float *power_mw) {
    if (!power_mw || !s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t raw_val;
    esp_err_t ret = read_reg(REG_POWER, &raw_val);
    if (ret == ESP_OK) *power_mw = (float)raw_val * s_power_lsb * 1000.0f;
    return ret;
}

/**
 * Perform a full capture of power metrics and optionally trigger safety alerts.
 */
esp_err_t ina_read(ina_data_t *data, bool check_alerts) {
    if (!data) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ina_read_voltage(&data->voltage_mv);
    ret |= ina_read_current(&data->current_ma);
    ret |= ina_read_power(&data->power_mw);
    
    if (check_alerts) ina_check_alerts();
    return (ret == ESP_OK) ? ESP_OK : ESP_FAIL;
}

// =============================================================================
// Public API: Safety
// =============================================================================

/**
 * Evaluate current power metrics against safety thresholds.
 * Triggers audible alerts and warning logs if thresholds are breached.
 */
esp_err_t ina_check_alerts(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

#ifdef CONFIG_INA226_ALERTS_ENABLED
    static uint32_t last_alert_time = 0;
    uint32_t now_sec = (uint32_t)(esp_timer_get_time() / 1000000);
    if ((now_sec - last_alert_time) < ALERT_COOLDOWN_SEC) return ESP_OK;

    float voltage_mv = 0, current_ma = 0;
    ina_read_voltage(&voltage_mv);
    ina_read_current(&current_ma);

    bool triggered = false;
    if (voltage_mv < CONFIG_INA226_ALERT_VOLTAGE_THRESHOLD_MV) {
        ESP_LOGW(TAG, "Low Battery: %.1f mV", voltage_mv);
        audio_player_play(LOW_VOLTAGE_SOUND);
        triggered = true;
    } else if (current_ma > CONFIG_INA226_ALERT_CURRENT_THRESHOLD_MA) {
        ESP_LOGW(TAG, "High Current: %.1f mA", current_ma);
        audio_player_play(HIGH_CURRENT_SOUND);
        triggered = true;
    }
    if (triggered) last_alert_time = now_sec;
#endif

    return ESP_OK;
}
