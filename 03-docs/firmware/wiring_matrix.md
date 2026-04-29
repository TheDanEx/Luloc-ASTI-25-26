# Wiring Matrix (Matriz de Conexiones Hardware)

## Propósito Arquitectónico
Este documento actúa como el "Ground Truth" del hardware. Define la asignación de pines GPIO del ESP32-P4 para evitar colisiones entre periféricos y facilitar el mantenimiento del robot.

## Comparativa de Reasignación (Antes vs Después)

| Función | Antes (Config. Temporal) | Después (Nueva Tabla 28/04) | Notas Críticas |
| :--- | :--- | :--- | :--- |
| **Monitor Serie** | USB-OTG Interno | **GPIO 1** | UART0 TX (Chip CH343) |
| **Encoder 1 (IZQ)** | 33, 46 | **2, 3** | PCNT Unit 0 |
| **Encoder 2 (DER)** | 27, 32 | **4, 5** | PCNT Unit 1 |
| **Motor 1 (IZQ)** | 22, 23 | **6, 15** | MCPWM Gen 0 |
| **Motor 2 (DER)** | 21, 20 | **26, 27** | MCPWM Gen 1 |
| **Bus I2C (Audio/INA)** | 7, 8 | **7, 8** | Sin cambios |
| **Activación Sensores** | 54 (Previo) | **11** | Pin liberado de MicroSD |
| **Lidar (RX)** | - | **14** | Solo recepción de datos |
| **Sensores Línea (S1-S8)**| 16-23 | **16-23** | ADC1_CH0 a ADC1_CH7 |

## Distribución Detallada de Pines

| GPIO | Función | Protocolo | Notas |
| :--- | :--- | :--- | :--- |
| **1** | UART0 TX | Debug | Reservado depuración serie |
| **2** | Encoder 1A | PCNT | Entrada digital |
| **3** | Encoder 1B | PCNT | Entrada digital |
| **4** | Encoder 2A | PCNT | Entrada digital |
| **5** | Encoder 2B | PCNT | Entrada digital |
| **6** | Motor 1A | MCPWM | Control Motor Izquierdo |
| **7** | I2C SDA | I2C | Audio ES8311 + INA226 |
| **8** | I2C SCL | I2C | Audio ES8311 + INA226 |
| **11** | IR_EN | Digital Out | Activación Sensores (MOSFET) |
| **14** | Lidar RX | UART RX | Lectura sensor distancia |
| **15** | Motor 1B | MCPWM | Control Motor Izquierdo |
| **16** | Sensor 1 | ADC1_CH0 | Line Sensor Array |
| **17** | Sensor 2 | ADC1_CH1 | Line Sensor Array |
| **18** | Sensor 3 | ADC1_CH2 | Line Sensor Array |
| **19** | Sensor 4 | ADC1_CH3 | Line Sensor Array |
| **20** | Sensor 5 | ADC1_CH4 | Line Sensor Array |
| **21** | Sensor 6 | ADC1_CH5 | Line Sensor Array |
| **22** | Sensor 7 | ADC1_CH6 | Line Sensor Array |
| **23** | Sensor 8 | ADC1_CH7 | Line Sensor Array |
| **26** | Motor 2A | MCPWM | Control Motor Derecho |
| **27** | Motor 2B | MCPWM | Control Motor Derecho |

## Puntos Críticos y Depuración
- **Pines ADC1 (16-23):** Se mantienen como canales nativos para máxima velocidad de muestreo.
- **MicroSD:** Los pines 11 y 14 han sido liberados del bus MicroSD para su uso en sensores y Lidar.
- **Motores:** Se ha verificado que los pines 6, 15, 26 y 27 son compatibles con MCPWM y no interfieren con el arranque del chip.
