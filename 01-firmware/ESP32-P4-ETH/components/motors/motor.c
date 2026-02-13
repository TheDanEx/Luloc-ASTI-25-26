#include "motor.h"
#include "esp_log.h"

static const char *TAG = "motor_mcpwm";

static inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// MCPWM legacy usa duty en porcentaje (float): 0..100
static inline float map_1000_to_percent(int16_t v_abs) {
    // v_abs: 0..1000
    return (float)v_abs * 100.0f / 1000.0f;
}

static void set_motor_one(const motor_driver_mcpwm_t *m, const motor_hw_mcpwm_t *hw, int16_t cmd) {
    int16_t v = clamp_i16(cmd, -1000, 1000);

    // deadband
    if (m->deadband > 0 && (v < m->deadband && v > -m->deadband)) {
        v = 0;
    }

    if (v == 0) {
        // stop según política
        if (m->brake_on_stop) {
            // Brake: IN1=IN2=1 (aprox con duty 100%)
            mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_A, 100.0f);
            mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_B, 100.0f);
        } else {
            // Coast: IN1=IN2=0
            mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_A, 0.0f);
            mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_B, 0.0f);
        }
        return;
    }

    float duty = map_1000_to_percent((int16_t)(v < 0 ? -v : v));

    if (v > 0) {
        // Forward: IN1 PWM (OPR_A), IN2 = 0 (OPR_B)
        mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_A, duty);
        mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_B, 0.0f);
    } else {
        // Reverse: IN1 = 0, IN2 PWM
        mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(hw->unit, hw->timer, MCPWM_OPR_B, duty);
    }

    // Duty mode “normal”
    mcpwm_set_duty_type(hw->unit, hw->timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(hw->unit, hw->timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

bool motor_mcpwm_init(motor_driver_mcpwm_t *m) {
    if (!m) return false;

    // nSLEEP opcional
    if (m->nsleep != GPIO_NUM_NC) {
        gpio_reset_pin(m->nsleep);
        gpio_set_direction(m->nsleep, GPIO_MODE_OUTPUT);
        gpio_set_level(m->nsleep, 1);
    }

    // LEFT GPIO routing
    if (m->left.timer == MCPWM_TIMER_0) {
        mcpwm_gpio_init(m->left.unit, MCPWM0A, m->left.in1);
        mcpwm_gpio_init(m->left.unit, MCPWM0B, m->left.in2);
    } else if (m->left.timer == MCPWM_TIMER_1) {
        mcpwm_gpio_init(m->left.unit, MCPWM1A, m->left.in1);
        mcpwm_gpio_init(m->left.unit, MCPWM1B, m->left.in2);
    } else {
        mcpwm_gpio_init(m->left.unit, MCPWM2A, m->left.in1);
        mcpwm_gpio_init(m->left.unit, MCPWM2B, m->left.in2);
    }

    // RIGHT GPIO routing
    if (m->right.timer == MCPWM_TIMER_0) {
        mcpwm_gpio_init(m->right.unit, MCPWM0A, m->right.in1);
        mcpwm_gpio_init(m->right.unit, MCPWM0B, m->right.in2);
    } else if (m->right.timer == MCPWM_TIMER_1) {
        mcpwm_gpio_init(m->right.unit, MCPWM1A, m->right.in1);
        mcpwm_gpio_init(m->right.unit, MCPWM1B, m->right.in2);
    } else {
        mcpwm_gpio_init(m->right.unit, MCPWM2A, m->right.in1);
        mcpwm_gpio_init(m->right.unit, MCPWM2B, m->right.in2);
    }

    // Config común (la aplicas por timer)
    mcpwm_config_t cfg = {
        .frequency = (uint32_t)m->pwm_hz,
        .cmpr_a = 0.0f,                // duty inicial A
        .cmpr_b = 0.0f,                // duty inicial B
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    if (mcpwm_init(m->left.unit, m->left.timer, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_init failed (left)");
        return false;
    }

    if (mcpwm_init(m->right.unit, m->right.timer, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_init failed (right)");
        return false;
    }

    motor_mcpwm_stop(m);
    ESP_LOGI(TAG, "MCPWM init OK (pwm=%lu Hz)", (unsigned long)m->pwm_hz);
    return true;
}

void motor_mcpwm_set(motor_driver_mcpwm_t *m, int16_t left, int16_t right) {
    if (!m) return;
    set_motor_one(m, &m->left, left);
    set_motor_one(m, &m->right, right);
}

void motor_mcpwm_stop(motor_driver_mcpwm_t *m) {
    if (!m) return;
    motor_mcpwm_set(m, 0, 0);
}

void motor_mcpwm_brake(motor_driver_mcpwm_t *m) {
    if (!m) return;
    bool prev = m->brake_on_stop;
    m->brake_on_stop = true;
    motor_mcpwm_stop(m);
    m->brake_on_stop = prev;
}

void motor_mcpwm_coast(motor_driver_mcpwm_t *m) {
    if (!m) return;
    // Coast: ambos a 0
    mcpwm_set_duty(m->left.unit,  m->left.timer,  MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(m->left.unit,  m->left.timer,  MCPWM_OPR_B, 0.0f);
    mcpwm_set_duty(m->right.unit, m->right.timer, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(m->right.unit, m->right.timer, MCPWM_OPR_B, 0.0f);
}

void motor_mcpwm_sleep(motor_driver_mcpwm_t *m, bool sleep) {
    if (!m) return;
    if (m->nsleep == GPIO_NUM_NC) return;

    gpio_set_level(m->nsleep, sleep ? 0 : 1);
    if (sleep) {
        motor_mcpwm_coast(m);
    }
}
