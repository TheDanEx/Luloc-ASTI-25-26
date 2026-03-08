# Componente: PID Tuner & NVS Manager (`pid_tuner`)

## Propósito Arquitectónico
Los robots diferenciales modernos en entornos de fricción real sufren de una ineficacia notoria con algoritmos de "Autotune" puro. La arquitectura estándar de la industria exige sintonizar en vivo mientas el robot está en su entorno de trabajo.
El componente `pid_tuner` sirve como un puente seguro y asíncrono entre:
1. **MQTT (Input):** Recibe comandos JSON generados por el usuario desde un Dashboard externo.
2. **Memoria NVS (Persistencia):** Guarda las ganancias instantáneamente en la Flash del ESP32 para que sobrevivan a reinicios y caídas de tensión.
3. **Memoria Compartida (Output):** Inyecta las variables en RAM para que el lazo de Alta Frecuencia (`task_rtcontrol_cpu0`) las absorba bajo demanda sin bloquearse jamás parseando JSONs o leyendo EEPROM.

## Entorno y Dependencias
- **Framework:** ESP-IDF v5.5.
- **Hardware Agnostico:** Utiliza `nvs_flash` y abstracciones de memoria compartida.
- **Dependencias Locales:** Componentes `cJSON`, `mqtt_custom_client`, `shared_memory`.

## Interfaces de E/S (Inputs/Outputs)
### Input (Suscripción MQTT):
Escucha en el tópico definido en menú `Kconfig` (por defecto `robot/config/pid_motors`).
Espera un Payload JSON (que puede ser total o parcial):
```json
{"kp": 0.5, "ki": 0.05, "kd": 0.01}
```
*Si pasas solo `{"kp": 0.6}`, el componente es lo suficientemente inteligente como para abrir la partición Flash NVS, rescatar el `ki` y `kd` almacenado préviamente, combinarlos y volver a guardarlos para no destruir datos.*

### Output Lógico (`shared_memory.h`):
Exporta silenciosamente a la RAM protegida por Mutex una bandera booleana `shm->live_pid.updated_flag`.

## Flujo de Ejecución Lógico
1. El sistema arranca `task_comms_cpu1` (Hebra de Baja Prioridad / WiFi).
2. Se llama a `pid_tuner_init()`, que monta o formatea si es necesario la partición `nvs`.
3. El módulo inscribe la función Callback al gestor MQTT principal.
4. Cuando el usuario envía un payload por WiFi, el Callback parsea el JSON, guarda la data resultante al Storage Persistente bloqueando momentáneamente esa hebra, e inyecta la bandera en la Estructura Compartida.
5. De forma paralela, el control RTOS rápido de motores lee esa bandera en el siguiente milisegundo e invoca la recarga del objeto matemático sin pestañear.

## Funciones Principales y Parámetros
- `esp_err_t pid_tuner_init(void)`: Inicializa NVS.
- `esp_err_t pid_tuner_load_motor_pid(float *kp, float *ki, float *kd)`: Rellena los punteros con los valores residentes en Flash.
- `esp_err_t pid_tuner_save_motor_pid(...)`: Realiza un Flush seguro a NVS.
- `esp_err_t pid_tuner_subscribe(void)`: Obliga al cliente MQTT a escuchar la API.

## Puntos Críticos y Depuración
- **Partición Fantasma:** Al almacenar floats, internamente se castean a pelo a RAW bitwise de 32 bits y se guardan como un Entero de 32 bits genérico debido a las limitaciones de ESP-IDF nativo. Esto ha sido verificado que funciona con soltura y es invisible desde la API exterior.
- **No Invoques Reads NVS en Lazo:** El usuario de la API jamás debe llamar a `pid_tuner_load_motor_pid` repetitivamente de forma cíclica. Esa función congela el RTOS docenas de milisegundos bajando al bus SPI de la placa. Hazlo sólo durante tu setup en el `app_main!` y luego fíjate exclusivamente en la memoria RAM `shared_memory.h`.

## Ejemplo de Uso e Instanciación
```c
#include "pid_tuner.h"
#include "motor_velocity_ctrl.h"
#include "shared_memory.h"

// ---------------------------------------------------------------------
// LADO DE RED (CPU1)
// ---------------------------------------------------------------------
void task_comms(void* arg) {
    pid_tuner_init();
    pid_tuner_register_callback();
    
    // Mientras la red esté viva...
    pid_tuner_subscribe();
}

// ---------------------------------------------------------------------
// LADO DE CONTROL TIEMPO-REAL (CPU0)
// ---------------------------------------------------------------------
void task_motors(void* arg) {
    motor_velocity_config_t m_config = { .kp=0.1, .ki=0, .kd=0 }; // Base temporal
    
    // Al inicializar el CPU0, tiramos del Disco Duro (NVS) para arrancar 
    // recordando la sesión anterior
    pid_tuner_load_motor_pid(&m_config.kp, &m_config.ki, &m_config.kd);
    motor_velocity_ctrl_handle_t ctrl_left;
    motor_velocity_ctrl_create(&m_config, &ctrl_left);

    while(1) {
        shared_memory_t* shm = shared_memory_get();
        xSemaphoreTake(shm->mutex, portMAX_DELAY);

        // ¡Avisa si por MQTT acaba de llegar un tuning live!
        if (shm->live_pid.updated_flag) {
            motor_velocity_ctrl_set_pid(ctrl_left, 
                shm->live_pid.kp, shm->live_pid.ki, shm->live_pid.kd);
            
            shm->live_pid.updated_flag = false; // Consumido
        }
        xSemaphoreGive(shm->mutex);

        // [...] Update Matemático [...]
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```
