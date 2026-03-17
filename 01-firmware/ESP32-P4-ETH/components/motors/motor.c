#include "motor.h"
#include "esp_log.h"

static const char *TAG = "motor_mcpwm";

static inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline uint32_t map_1000_to_ticks(uint16_t v_abs, uint32_t period_ticks) {
    // v_abs: 0..1000  ->  0..period_ticks
    return (uint32_t)((uint64_t)v_abs * (uint64_t)period_ticks / 1000ULL);
}

static esp_err_t setup_one_motor(motor_driver_mcpwm_t *m, motor_hw_mcpwm_t *hw, int group_id)
{
    // 1) Crear operador en el grupo (normalmente group 0)
    mcpwm_operator_config_t oper_cfg = {
        .group_id = group_id,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &hw->oper));

    // 2) Conectar operador al timer compartido
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(hw->oper, m->timer));

    // 3) Crear comparadores (uno por IN1 y otro por IN2)
    //    update_cmp_on_tez: actualiza el compare cuando el timer llega a 0 (TEZ)
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = 1,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(hw->oper, &cmp_cfg, &hw->cmpr_in1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(hw->oper, &cmp_cfg, &hw->cmpr_in2));

    // 4) Crear generadores (y asignar GPIO físicos)
    mcpwm_generator_config_t gen_cfg = {0};

    gen_cfg.gen_gpio_num = hw->in1;
    ESP_ERROR_CHECK(mcpwm_new_generator(hw->oper, &gen_cfg, &hw->gen_in1));

    gen_cfg.gen_gpio_num = hw->in2;
    ESP_ERROR_CHECK(mcpwm_new_generator(hw->oper, &gen_cfg, &hw->gen_in2));

    // 5) Definir forma de onda PWM:
    //    - Al inicio del periodo (timer = 0, contando UP): poner HIGH
    //    - Al llegar al compare: poner LOW
    //
    // Esto produce un pulso HIGH de anchura "compare" ticks.

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        hw->gen_in1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        hw->gen_in1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, hw->cmpr_in1, MCPWM_GEN_ACTION_LOW)
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        hw->gen_in2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        hw->gen_in2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, hw->cmpr_in2, MCPWM_GEN_ACTION_LOW)
    ));

    // Arrancamos “apagado”: fuerza a LOW ambos pines
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(hw->gen_in1, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(hw->gen_in2, 0, true));

    return ESP_OK;
}

// Helpers de “force”
static inline void gen_force_low(mcpwm_gen_handle_t gen)  { (void)mcpwm_generator_set_force_level(gen, 0, true); }
static inline void gen_force_high(mcpwm_gen_handle_t gen) { (void)mcpwm_generator_set_force_level(gen, 1, true); }
// Quitar force: -1 (según API / ejemplos de Espressif)
static inline void gen_release(mcpwm_gen_handle_t gen)    { (void)mcpwm_generator_set_force_level(gen, -1, true); }

static void set_motor_one(motor_driver_mcpwm_t *m, motor_hw_mcpwm_t *hw, int16_t cmd)
{
    int16_t v = clamp_i16(cmd, -1000, 1000);

    // Deadband check
    if (m->deadband > 0 && (v < m->deadband && v > -m->deadband)) {
        v = 0;
    }

    if (v == 0) {
        if (m->brake_on_stop) {
            // Brake: IN1=IN2=1
            gen_force_high(hw->gen_in1);
            gen_force_high(hw->gen_in2);
        } else {
            // Coast: IN1=IN2=0
            gen_force_low(hw->gen_in1);
            gen_force_low(hw->gen_in2);
        }
        return;
    }

    uint16_t v_abs = (uint16_t)(v < 0 ? -v : v);
    uint32_t duty_ticks = map_1000_to_ticks(v_abs, m->period_ticks);

    if (duty_ticks == 0) {
        if (m->brake_on_stop) {
            gen_force_high(hw->gen_in1);
            gen_force_high(hw->gen_in2);
        } else {
            gen_force_low(hw->gen_in1);
            gen_force_low(hw->gen_in2);
        }
        return;
    }

    // --- SLOW DECAY (Brake-Mode PWM) Implementation ---
    // For DRV8871:
    // H, L = Forward
    // L, H = Reverse
    // H, H = Brake (Slow Decay)
    // L, L = Coast (Fast Decay)
    //
    // To achieve Slow Decay during PWM:
    // Forward: IN1 is kept HIGH, IN2 toggles PWM.
    // PWM cycle: IN2=L (Drive) -> IN2=H (Brake)
    
    if (v > 0) {
        // Forward
        gen_force_high(hw->gen_in1);
        gen_release(hw->gen_in2);
        
        // Since generator setup is TEZ=HIGH, CMP=LOW (Active High):
        // duty_ticks ticks of HIGH = duty_ticks ticks of BRAKE.
        // We want duty_ticks of DRIVE (L).
        // So we set compare to (period - duty_ticks).
        mcpwm_comparator_set_compare_value(hw->cmpr_in2, m->period_ticks - duty_ticks);
    } else {
        // Reverse
        gen_force_high(hw->gen_in2);
        gen_release(hw->gen_in1);
        
        mcpwm_comparator_set_compare_value(hw->cmpr_in1, m->period_ticks - duty_ticks);
    }
}

esp_err_t motor_mcpwm_init(motor_driver_mcpwm_t *m)
{
    if (!m) return ESP_ERR_INVALID_ARG;

    // nSLEEP opcional
    if (m->nsleep != GPIO_NUM_NC) {
        gpio_reset_pin(m->nsleep);
        gpio_set_direction(m->nsleep, GPIO_MODE_OUTPUT);
        gpio_set_level(m->nsleep, 1);
    }

    // Defaults razonables si no los has puesto
    if (m->resolution_hz == 0) m->resolution_hz = 10 * 1000 * 1000; // 10 MHz
    if (m->pwm_hz == 0)        m->pwm_hz = 20000;                    // 20 kHz

    m->period_ticks = m->resolution_hz / m->pwm_hz;
    if (m->period_ticks < 10) {
        ESP_LOGW(TAG, "period_ticks muy bajo (%lu). Sube resolution_hz o baja pwm_hz",
                 (unsigned long)m->period_ticks);
    }

    // 1) Crear timer MCPWM
    mcpwm_timer_config_t tcfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = m->resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = m->period_ticks,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &m->timer));

    // 2) Crear recursos para cada motor (operador+gens+comparadores)
    ESP_ERROR_CHECK(setup_one_motor(m, &m->left,  0));
    ESP_ERROR_CHECK(setup_one_motor(m, &m->right, 0));

    // 3) Enable + start del timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(m->timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_START_NO_STOP));

    motor_mcpwm_stop(m);

    ESP_LOGI(TAG, "MCPWM(prelude) init OK: pwm=%lu Hz, res=%lu Hz, period=%lu ticks",
             (unsigned long)m->pwm_hz,
             (unsigned long)m->resolution_hz,
             (unsigned long)m->period_ticks);

    return ESP_OK;
}

void motor_mcpwm_set(motor_driver_mcpwm_t *m, int16_t left, int16_t right)
{
    if (!m) return;
    set_motor_one(m, &m->left, left);
    set_motor_one(m, &m->right, right);
}

void motor_mcpwm_stop(motor_driver_mcpwm_t *m)
{
    if (!m) return;
    motor_mcpwm_set(m, 0, 0);
}

void motor_mcpwm_coast(motor_driver_mcpwm_t *m)
{
    if (!m) return;
    gen_force_low(m->left.gen_in1);
    gen_force_low(m->left.gen_in2);
    gen_force_low(m->right.gen_in1);
    gen_force_low(m->right.gen_in2);
}

void motor_mcpwm_brake(motor_driver_mcpwm_t *m)
{
    if (!m) return;
    gen_force_high(m->left.gen_in1);
    gen_force_high(m->left.gen_in2);
    gen_force_high(m->right.gen_in1);
    gen_force_high(m->right.gen_in2);
}

void motor_mcpwm_sleep(motor_driver_mcpwm_t *m, bool sleep)
{
    if (!m) return;
    if (m->nsleep == GPIO_NUM_NC) return;

    if (sleep) {
        motor_mcpwm_coast(m);
        gpio_set_level(m->nsleep, 0);
    } else {
        gpio_set_level(m->nsleep, 1);
    }
}
