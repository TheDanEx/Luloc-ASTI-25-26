## **Módulo 1: La Arquitectura del Cerebro (Microprocesador vs. Microcontrolador)**

Muchos estudiantes confunden la Raspberry Pi con un Arduino (o ESP32). Parecen lo mismo, pero son bestias opuestas. Para nuestro robot, usamos una **Arquitectura Híbrida**.

### **1.1. La Diferencia Fundamental: Determinismo**

* **Microprocesador (MPU) \- Raspberry Pi 5:** Es potente. Tiene Gigahercios y mucha RAM. Corre un Sistema Operativo (Linux). Pero Linux es *multitarea*. Mientras intenta leer un sensor, puede decidir actualizar un archivo de log o gestionar el WiFi. Esto introduce **Jitter** (retraso variable).  
  * *Problema:* Si el robot va a 2 m/s y el Linux se "distrae" 50ms, habrás recorrido 10 cm a ciegas.  
* **Microcontrolador (MCU) \- ESP32-P4 / STM32:** Es más lento (Megahercios) y tiene poca RAM. Pero es **Determinista**. Ejecuta un código en bucle sin interrupciones del sistema operativo.  
  * *Solución:* Si le pides leer un encoder cada 1ms, lo hará cada 1000 µs exactos.

### **1.2. ¿Por qué hemos elegido esta combinación?**

**Decisión de Diseño:** Delegamos funciones para garantizar seguridad y potencia.

1. **ESP32-P4 (El Cerebelo):** Se encarga de los "reflejos". Lee encoders, IMU y controla los motores. Si la Raspberry se cuelga, el ESP32 puede frenar el robot. Además, es barato (\~18€).  
2. **Raspberry Pi 5 (El Lóbulo Frontal):** Se encarga de la estrategia. Procesa la cámara, corre la IA y decide "hacia dónde ir". Envía órdenes de alto nivel al ESP32 ("Avanza a 1 m/s").

## ---

**Módulo 2: Actuadores \- La Fuerza Motriz**

### **2.1. Motores DC vs. Servos Inteligentes (Dynamixel)**

Aquí es donde el presupuesto y la aplicación definen el componente.

| Característica | Motores DC (JGA25-370) con Reductora | Servos Inteligentes (Dynamixel) |
| :---- | :---- | :---- |
| **Control** | "Tonto". Necesitas encoders externos y un PID propio. | Inteligente. Tienen PID, encoder y comunicaciones integrados. |
| **Velocidad** | **Alta.** Ideal para ruedas y carreras. | Baja/Media. Optimizados para posición (brazos robóticos). |
| **Giro** | Continuo (rotación infinita natural). | Generalmente restringido (aunque tienen modo rueda). |
| **Precio** | **Económico (\~10-15€).** | Caro (40€ \- 300€ por unidad). |

¿Por qué elegimos DC JGA25?  
Para un robot móvil de competición (tipo Sumo o Carreras), necesitamos velocidad y par continuo a bajo coste. Los Dynamixel son increíbles para un robot caminante o un brazo manipulador donde la precisión de posición (grados exactos) es más importante que las RPM constantes. Con un presupuesto de 250€, gastar 100€ en dos motores Dynamixel nos dejaría sin dinero para sensores.

### **2.2. El Driver (TB9051FTG)**

El microcontrolador funciona a 3.3V y apenas da unos miliamperios. Los motores necesitan 12V y varios amperios. Si conectas el motor directo al ESP32, **lo quemas al instante**.

* **Función:** El driver actúa como un grifo digital. El ESP32 abre/cierra el grifo (señal PWM) y el driver deja pasar la corriente de la batería a los motores.

## ---

**Módulo 3: Electricidad \- Energía y Seguridad**

### **3.1. Serie vs. Paralelo (La Regla de Oro)**

Para entender vuestra batería LiPo 4S, debéis entender cómo se suman las celdas.

* **En Serie (Sumamos Voltaje \- Tensión):**  
  * Conectamos el positivo de una celda al negativo de la siguiente.  
  * *Analogía:* Es como poner bombas de agua una encima de otra. Tienes más presión (Voltaje) para mover el agua.  
  * **Caso Robot:** 4 celdas de 3.7V en serie \= $3.7V \\times 4 \= 14.8V$.  
  * *¿Por qué?* Los motores necesitan 12V para dar su máximo par. Con menos voltaje, el robot sería lento y débil en el Sumo.  
* **En Paralelo (Sumamos Amperaje \- Capacidad/Duración):**  
  * Conectamos positivos con positivos y negativos con negativos.  
  * *Analogía:* Es como poner tuberías más anchas. Tienes el mismo empuje, pero cabe más agua (dura más tiempo).  
  * **Caso Robot:** Si pusiéramos las 4 celdas en paralelo, tendríamos 3.7V pero con 4 veces más duración. *Inútil* porque el voltaje no mueve el motor.

### **3.2. Seguridad Crítica**

**Advertencia:** Una LiPo 4S descargada por debajo de 12.8V se daña químicamente. Usamos el sensor **INA226** no por capricho, sino para leer el voltaje en tiempo real y que el robot se apague solo si la batería está baja.

## ---

**Módulo 4: El Arte del Control \- PID (Proporcional, Integral, Derivativo)**

Aquí está la magia. Un robot sin PID es como conducir borracho: das volantazos y nunca vas recto. Un PID es un algoritmo matemático que corrige el error continuamente.

La fórmula básica en tiempo continuo es:

$$u(t) \= K\_p e(t) \+ K\_i \\int e(t) dt \+ K\_d \\frac{de(t)}{dt}$$  
Donde $e(t)$ es el **Error** (Diferencia entre lo que quieres y lo que tienes).

### **4.1. Explicación Práctica de los Términos**

Imagina que el robot debe seguir una línea (Target \= Centro).

1. **P (Proporcional) \- El Presente:** "Estoy lejos, giro fuerte".  
   * Si el error es grande, la corrección es grande.  
   * *Defecto:* Si el error es pequeño, la corrección es mínima y a veces no llega al objetivo exacto.  
2. **I (Integral) \- El Pasado:** "Llevo rato desviado, voy a empujar más".  
   * Acumula el error en el tiempo. Si el robot se queda atascado un poco a la izquierda, la "I" va creciendo hasta que fuerza al robot a corregir. Elimina el error estacionario.  
3. **D (Derivativo) \- El Futuro:** "Me estoy acercando muy rápido, voy a frenar antes de pasarme".  
   * Predice el error futuro basándose en la velocidad de cambio. Evita que el robot oscile (vibre) alrededor de la línea.

### **4.2. Escenarios de uso en vuestro Robot**

#### **A. PID de Velocidad (Cruise Control)**

* **Objetivo:** Queremos que la rueda gire a 10 rad/s, suba o baje una rampa.  
* **Sensor:** Encoder (mide velocidad real).  
* **Actuador:** PWM del Motor.  
* *Sin PID:* En una subida, el robot se frena.  
* *Con PID:* Al detectar que la velocidad baja (Error positivo), el PID aumenta la potencia (PWM) automáticamente para mantener los 10 rad/s.

#### **B. PID de Orientación (Giroscopio/IMU)**

* **Objetivo:** Mantener el robot recto (0 grados) tras un choque.  
* **Sensor:** IMU BNO085.  
* **Acción:** Si el robot recibe un golpe y gira a 10°, el PID detecta el error y manda girar las ruedas en sentido contrario para recuperar los 0°.

### **4.3. Ejemplo de Programación (Pseudocódigo C++ para ESP32)**

Este es un ejemplo real de cómo se implementa un PID para controlar la velocidad de un motor.

C++

// Variables globales (Ajustar estas K es el "Tuning")  
float Kp \= 2.0;  // Fuerza de reacción inmediata  
float Ki \= 0.5;  // Fuerza para corregir errores acumulados  
float Kd \= 0.1;  // Freno para evitar oscilaciones

float error\_previo \= 0;  
float integral \= 0;

// Esta función se debe llamar en un bucle fijo (ej. cada 10ms \-\> dt \= 0.01)  
// target\_speed: Velocidad deseada (m/s)  
// current\_speed: Velocidad leída por el encoder (m/s)  
int calcular\_PWM\_PID(float target\_speed, float current\_speed, float dt) {  
      
    // 1\. Calcular el Error  
    float error \= target\_speed \- current\_speed;  
      
    // 2\. Término Proporcional  
    float P \= Kp \* error;  
      
    // 3\. Término Integral (Suma de errores)  
    integral \= integral \+ (error \* dt);  
    float I \= Ki \* integral;  
      
    // 4\. Término Derivativo (Velocidad de cambio del error)  
    float derivada \= (error \- error\_previo) / dt;  
    float D \= Kd \* derivada;  
      
    // 5\. Calcular salida  
    float salida \= P \+ I \+ D;  
      
    // Guardar error para la siguiente vuelta  
    error\_previo \= error;  
      
    // Limitar la salida al rango del PWM (ej. 0 a 255\)  
    if (salida \> 255) salida \= 255;  
    if (salida \< \-255) salida \= \-255; // Para motor reversible  
      
    return (int)salida;  
}

## ---

**Módulo 5: Sensores y Comunicación (¿Por qué estos?)**

### **5.1. LiDAR vs. Ultrasonidos**

Elegimos **LiDAR (D200/X4Pro)** porque usa luz láser.

* **Comparativa:** Un sensor de ultrasonidos es sordo y lento (la velocidad del sonido es lenta y el eco rebota mal en esquinas). El LiDAR nos da 360 puntos de distancia instantáneos. Necesitamos esto para hacer mapas (SLAM).

### **5.2. InfluxDB (La Caja Negra)**

¿Por qué no guardar los datos en un .txt o MySQL?

* Porque vuestro robot genera 100 datos por segundo (telemetría). MySQL es lenta para escribir datos tan rápido. InfluxDB está diseñada para **Series Temporales**. Nos permite ver en una gráfica, segundo a segundo, por qué el PID falló en la curva 3 analizando el voltaje, la velocidad y el error simultáneamente.

### ---

**Resumen para el Estudiante**

1. **ESP32** para controlar los músculos (motores) en tiempo real.  
2. **Raspberry Pi** para pensar la estrategia.  
3. **Batería en Serie** para tener fuerza (Voltaje).  
4. **Motores DC** para correr rápido y barato.  
5. **PID** para que el robot obedezca y no oscile.

**Módulo 6: Anatomía Interna del Control (ESP32-P4)**

No habéis elegido un microcontrolador cualquiera. El ESP32-P4 es una bestia asimétrica. Debéis programarlo entendiendo su **dualidad**:

### **6.1. HP System vs. LP System**

* **HP System (High Performance \- RISC-V Dual Core @ 400MHz):**  
  * *Función:* Es el "Jefe de Obra". Maneja la comunicación Ethernet pesada, los cálculos trigonométricos del LiDAR y el algoritmo principal de movimiento.  
  * *Por qué:* Necesitamos potencia bruta para procesar datos a alta velocidad sin bloquear el bucle de control.  
* **LP System (Low Power \- RISC-V Single Core @ 40MHz):**  
  * *Función:* Es el "Vigilante Nocturno". Puede funcionar mientras el resto del chip duerme.  
  * *Aplicación Práctica:* Lo usaremos para leer el sensor **INA226** (batería). Si el voltaje cae peligrosamente, el LP System puede despertar al HP System a través del **LP Mailbox** para iniciar un apagado seguro.  
  * *Valor:* Seguridad redundante. Aunque el código principal se cuelgue (bug de software), el vigilante sigue activo.

### **6.2. MCPWM (Motor Control PWM)**

Olvidaos del analogWrite() de Arduino. Eso es para LEDs. Para motores usamos el periférico **MCPWM**.

3. **Diferencia Técnica:** Un PWM normal solo corta la señal. El MCPWM gestiona **Tiempos Muertos (Dead Time)**.  
4. *¿Por qué es crítico?* En un Puente H (el circuito dentro del driver), si activas el transistor de "avanzar" y "retroceder" al mismo tiempo por un microsegundo, creas un cortocircuito (Shoot-through) que quema el driver. El MCPWM inserta una pausa de seguridad nanosegunda automáticamente entre cambios de dirección.

## ---

**Módulo 7: Gestión de Energía (El Sistema Circulatorio)**

Vuestra batería LiPo 4S entrega **14.8V a 16.8V**. Si conectáis esto a la Raspberry Pi (que pide 5V), saldrá humo azul.

### **7.1. Conversión: XL4015 (Buck Converter)**

* **Tecnología:** Es un conversor **DC-DC Buck (Reductor)**.  
* **¿Por qué no un regulador lineal (L7805)?** Un lineal "quema" el voltaje sobrante como calor. De 14.8V a 5V, desperdiciaríamos el 65% de la energía en calor. El Buck "trocea" la energía inductivamente con una eficiencia del 90%+.  
* **El Factor 5A:** La Raspberry Pi 5 es caprichosa. Si el voltaje cae bajo 4.7V, entra en *Throttling* (se vuelve lenta) o se reinicia. El XL4015 garantiza 5 Amperios estables, separando el ruido eléctrico de los motores de la delicada lógica de la CPU.

### **7.2. Telemetría Energética: INA226**

* **Función:** Mide Voltaje ($V$) y Corriente ($A$) con precisión de 16-bits vía I2C.  
* **Diferencia con medir solo voltaje:** El voltaje de una LiPo baja *falsamente* cuando aceleras a fondo (V-drop). El INA226 nos permite leer la corriente y calcular la **Potencia Real ($W \= V \\times I$)** y los mAh consumidos (Coulomb Counting).  
* **Utilidad:** El robot sabrá decir: *"Me queda un 20% de batería real"*, en lugar de adivinarlo por el voltaje.

### **7.3. Cableado y Fusibles**

* **Fusibles 5x20mm:** Vuestro seguro de vida. Si los motores se bloquean en el Sumo, la corriente se disparará a \>11A. Sin fusible, la LiPo podría hincharse o arder. Con fusible, solo perdéis 10 céntimos y 1 minuto en cambiarlo.  
* **AWG (American Wire Gauge):**  
  * **16 AWG (Grueso):** Para Batería \-\> Driver \-\> Motor. *Por qué:* Menor resistencia. Queremos que la energía llegue a la rueda, no que se pierda calentando el cable.  
  * **24 AWG (Fino):** Para Sensores y Señales. *Por qué:* Ligeros y flexibles, fáciles de rutar por el chasis.

## ---

**Módulo 8: Percepción y Actuación Avanzada**

### **8.1. Matriz IR de 8 Canales (Siguelíneas)**

4. **Concepto:** No veáis esto como 8 interruptores. Es un sensor de **Error Analógico**.  
5. **Aplicación PID:**  
   * Si el sensor 4 y 5 ven negro: Error \= 0 (Centro).  
   * Si solo el sensor 8 ve negro: Error \= Max (Extremo derecho).  
   * Esto permite al algoritmo PID calcular *cuánto* girar, no solo *hacia dónde*. Permite tomar curvas suavemente sin frenar de golpe.

### **8.2. Drivers TB9051FTG y Feedback**

2. **Pin OCM (Output Current Monitor):** Este pin devuelve un voltaje análogo a la fuerza que hace el motor.  
3. **Estrategia de Sumo:** Si detectamos que la corriente sube (pico de amperios) pero los encoders dicen que la velocidad es 0, significa que **estamos empujando al oponente**. El ESP32 puede decidir entonces activar el "Modo Turbo" para ganar el empuje.  
4. **Pin DIAG:** Si el driver se calienta o falla, avisa al ESP32 para parar antes de quemarse permanentemente.

### **8.3. Servo MG996R (Actuación Secundaria)**

2. **Uso:** Mover mecanismos o golpear.  
3. **Advertencia:** Es una carga inductiva "sucia". Mete ruido en la línea de 5V. Aseguraos de que el condensador del Buck Converter esté cerca para absorber estos picos.

### **8.4. Visión Artificial (Raspberry Pi \+ IMX219)**

* **Flujo de Datos:**  
  * Cámara (MIPI-CSI) \-\> RPi 5 (Procesa imagen con OpenCV/ROS2).  
  * RPi 5 decide: "Enemigo a la derecha".  
  * Ethernet \-\> ESP32-P4 HP Core: "Comando: Girar derecha".  
  * ESP32-P4 \-\> Motores: Ejecuta el giro.  
* **Valor:** Permite estrategias complejas que los sensores simples no pueden ver (ej. diferenciar colores o leer códigos QR en pista).

## ---

**Módulo 9: Comunicaciones y Estructura**

### **9.1. La Autopista de Datos**

* **LiDAR (UART):** Envía distancias. Es rápido y simple punto a punto.  
* **Telemetría (Ethernet \+ MQTT \+ InfluxDB):**  
  * Usamos cable **Ethernet** entre Pi y ESP32 porque el WiFi tiene latencia (lag).  
  * Enviamos datos a **InfluxDB** (Base de datos de series temporales).  
  * *Ventaja:* Podemos graficar en el Dashboard la velocidad del motor vs. el voltaje de la batería milisegundo a milisegundo para diagnosticar problemas imposibles de ver a ojo.

### **9.2. Mecánica: PLA y Ruedas Locas**

6. **Caster Wheels (Ruedas Locas):** Vuestro robot tiene tracción diferencial (gira moviendo una rueda más que la otra). Necesita un tercer punto de apoyo que **no ofrezca resistencia** al giro. Una rueda fija arrastraría y arruinaría la odometría.  
7. **Chasis PLA:** Diseñado para evitar oclusiones al LiDAR. Si ponéis un cable delante del LiDAR, el robot pensará que siempre tiene una pared delante.

## ---

**Resumen Técnico del Sistema**

| Componente | Rol en el Sistema | Conectividad / Protocolo | Justificación Clave |
| :---- | :---- | :---- | :---- |
| **RPi 5** | IA, Visión y ROS2 | Maestro (Eth / MIPI-CSI) | Potencia de cómputo para visión, no determinista. |
| **ESP32-P4** | Control Tiempo Real | Esclavo (Eth / PWM / I2C) | Doble núcleo (HP/LP) para seguridad y determinismo. |
| **XL4015** | Conversión Energía | Potencia (Buck 14.8V \-\> 5V) | Eficiencia térmica y estabilidad ante picos de carga. |
| **TB9051FTG** | Driver Motores | PWM / DIR / OCM | Monitorización de corriente y protecciones HW. |
| **LiDAR X4** | Mapeo y Entorno | UART (128kbps) | Visión 360º rápida para evitar colisiones. |
| **BNO085** | Orientación (IMU) | I2C | Mantiene el rumbo recto corrigiendo deriva. |
| **INA226** | Salud Batería | I2C | Medición real de Ah consumidos (no solo Voltios). |
| **Matriz IR** | Sigue Líneas | GPIO / Analógico | Entrada de error granular para control PID fino. |

### 

