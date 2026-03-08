# Motors (MCPWM)

## Propósito Arquitectónico
Capa de abstracción hardware para manejar motores DC mediante modulación por ancho de pulsos de alta resolución (MCPWM). Diseñada explícitamente para puentes H en dual-channel (un canal Izquierdo y uno Derecho) como el Texas Instruments DRV8874.

## Entorno y Dependencias
Utiliza la moderna librería `driver/mcpwm_prelude.h` del ESP-IDF v5, abandonando drivers legados de PWM, proporcionando exactitud y alta eficiencia temporal.

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Pines digitales IN1 e IN2 para inyectores de señales directas de puerta del puente. Soporte opcional para pin digital nSLEEP.
- **Software:** Set-points asíncronos y no bloqueantes `motor_mcpwm_set(left, right)`. Parada (`stop`), freno activo continuo (`brake`) y frenado en vacío ('coast').

## Flujo de Ejecución Lógico
Configura en inicio los temporizadores y la frecuencia de ciclo de trabajo (e.g. 20kHz). Construye operadores, comparadores y generadores. Posteriormente en Run-Time únicamente se ajustan los umbrales de los comparadores mediante un wrapper muy rápido, alterando el Duty Cycle sobre la marcha.

## Funciones Principales y Parámetros
- `motor_mcpwm_init(motor_driver_mcpwm_t *m)`: Reserva recursos e inicializa los temporizadores del hardware para gobernar un driver entero de doble canal.
  - `m`: Puntero a la estructura maestra de configuración (configurada previamente por el usuario con los pines GPIO `in1`, `in2`, Hz, resolución, y deadband).
- `motor_mcpwm_set(motor_driver_mcpwm_t *m, int16_t left, int16_t right)`: Ajusta el ciclo de trabajo PWM de los dos motores al vuelo.
  - `m`: Puntero al manejador inicializado del driver del motor.
  - `left` / `right`: Valores numéricos de consigna de velocidad y dirección (Ej. `-1000` a `1000`, indicando sentido y fuerza).
- `motor_mcpwm_stop(motor_driver_mcpwm_t *m)` / `motor_mcpwm_coast(motor_driver_mcpwm_t *m)`: Apaga los pulsos PWM desconectando el puente H, dejando a los motores girar libremente hasta detenerse por arrastre (Alta impedancia).
- `motor_mcpwm_brake(motor_driver_mcpwm_t *m)`: Aplica un corto forzado en el puente (Activa ambas líneas al mismo voltaje LOW-LOW / HIGH-HIGH) induciendo frenado magnético abrupto.
- `motor_mcpwm_sleep(motor_driver_mcpwm_t *m, bool sleep)`: (Opcional) Bascula el pin `nSLEEP` del DRV8874 apagar el consumo base del controlador de potencia.
  - `sleep`: Boolean indicando entrada (`true`) o salida (`false`) del descanso.

## Puntos Críticos y Depuración
- **Transitorios de corriente destructivos:** La librería permite habilitar 'deadband' a nivel algoritmia (zona muerta cercana al valor 0) para evitar picos transitorios. Cuidar de no invertir de 100% adelante a 100% atrás bruscamente y hacer saltar protecciones del DRV.
- **Inexactitudes del freno:** Depende de si los pines van asimétricos LOW-LOW o HIGH-HIGH. Hay que validar configuraciones de brake contra coasting libres, vital para perfiles trapezoidal en robótica móvil.

## Ejemplo de Uso e Instanciación
```c
#include "motors.h"
#include "driver/gpio.h"

// 1. Declarar la configuración de los pines del DRV8874
motor_driver_mcpwm_t chasis_control = {
    .in1 = GPIO_NUM_18, // Pin PWM Directo
    .in2 = GPIO_NUM_19, // Pin PWM Inverso
    .pwm_freq_hz = 20000, // 20 kHz por encima del umbral audible
    .resolution_hz = 10000000, // Resolución interna timer 10MHz
    .deadband_ns = 50 // Para evitar cortos de puente completo
};

// 2. Tarea de control
void control_task(void *pvParameters) {
    // Inicializar temporizadores MCPWM y amarrar al handle
    esp_err_t err = motor_mcpwm_init(&chasis_control);
    
    // Despertar el chip de potencia si tiene pin SLEEP (Opcional)
    // motor_mcpwm_sleep(&chasis_control, false);

    while(1) {
        // Enviar velocidades asimétricas (Ej: -100% izquierda, +50% derecha)
        motor_mcpwm_set(&chasis_control, -1000, 500);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Ejecutar frenado electromagnético forzado (cortocircuito sobre H-Bridge)
        motor_mcpwm_brake(&chasis_control);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
