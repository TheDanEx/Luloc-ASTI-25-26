#include <Arduino.h>
#include <Preferences.h>

// --- CONFIGURACIÓN DE PINES ---
const int NUM_IR = 8;
const int ir_pins[NUM_IR] = {25, 27, 14, 12, 13, 15, 2, 4}; //Los pines conectados al sensor
const int PIN_ENABLE_IR = 33;

// Pines Motores (Ajusta estos a tu driver L298N o TB6612)
const int motorIzquierdaAdelante = 26;
const int motorIzquierdaAtras = 25;
const int motorDerechaAdelante = 32;
const int motorDerechaAtras = 19;

// --- VARIABLES DE CONTROL ---
bool MODO_CALIBRACION = false; 
Preferences pref;
int umbrales[NUM_IR];
int negro_calib[NUM_IR];
bool binary[NUM_IR];

// Configuración de velocidad
int velocidadBase = 150; // Velocidad de crucero (0-255)

// --- PROTOTIPOS DE FUNCIONES ---
void ejecutarCalibracion();
void cargarCalibracion();
void leerSensores();
float calcularError();
void controlMotores(int velIzquierda, int velDerecha);

void setup() {
    Serial.begin(115200);
    
    // Configuración Sensores
    pinMode(PIN_ENABLE_IR, OUTPUT);
    digitalWrite(PIN_ENABLE_IR, HIGH);
    for (int i = 0; i < NUM_IR; i++) pinMode(ir_pins[i], INPUT);

    // Configuración Motores
    pinMode(motorIzquierdaAdelante, OUTPUT);
    pinMode(motorIzquierdaAtras, OUTPUT);
    pinMode(motorDerechaAdelante, OUTPUT);
    pinMode(motorDerechaAtras, OUTPUT);

    pref.begin("robot_asti", false);

    if (MODO_CALIBRACION) {
        ejecutarCalibracion();
    } else {
        cargarCalibracion();
    }
}

void loop() {
    leerSensores();
    float error = calcularError();

    // Comprobamos si el robot ha perdido la línea (el valor especial que pusimos)
    if (error == -100) {
        // Si no hay línea, paramos los motores por seguridad
        controlMotores(0, 0); 
        Serial.println("!!! LÍNEA PERDIDA - EMERGENCIA !!!");
    } 
    else {
        // Si hay línea, calculamos la dirección normal
        int Kp = 5; // Ajusta este número: más alto = giro más brusco
        int correccion = (int)(error * Kp);

        int velI = velocidadBase + correccion; //SI LOS MOTORES FUNCIONAN AL REVES -> CAMBIAR SIGNO
        int velD = velocidadBase - correccion;

        controlMotores(velI, velD);

        // Debug para ver los datos en tiempo real en VS Code
        Serial.print("Error: "); Serial.print(error);
        Serial.print(" | Motores: "); Serial.print(velI);
        Serial.print(" - "); Serial.println(velD);
    }
    
    delay(10); // Esperamos 10ms para la siguiente decisión (100 veces por segundo)
}

// --- FUNCIONES CORE ---
void leerSensores() {
    for (int i = 0; i < NUM_IR; i++) {
        int raw = analogRead(ir_pins[i]);
        int limiteAire = negro_calib[i] * 1.15;
        if (limiteAire > 4050) limiteAire = 4050;
        
        // Aplicamos el margen de 250 que hablamos para los baches
        if (raw > limiteAire) binary[i] = 0; 
        else if (raw > (umbrales[i] + 250)) binary[i] = 1;
        else binary[i] = 0;
    }
}

float calcularError() {
    /* PESOS AJUSTADOS PARA LÍNEA DE 16mm:
       Damos valores que permitan una transición suave. 
       Los sensores centrales (S4, S5) tienen pesos bajos para evitar vibraciones.
    */
    int pesos[8] = {-15, -10, -5, -2, 2, 5, 10, 15}; 
    
    long sumaPesos = 0;
    int activos = 0;

    for (int i = 0; i < NUM_IR; i++) {
        if (binary[i]) {
            sumaPesos += pesos[i];
            activos++;
        }
    }

    // Si no hay sensores activos, el robot ha perdido la línea
    if (activos == 0) {
        return -100; // Usaremos este valor especial para activar una búsqueda
    }

    // Promedio ponderado: nos da la posición exacta del centro de la línea de 16mm
    return (float)sumaPesos / activos; 
}

void controlMotores(int velI, int velD) {
    // Limitamos las velocidades entre 0 y 255
    velI = constrain(velI, 0, 255);
    velD = constrain(velD, 0, 255);

    // Motor Izquierdo
    analogWrite(motorIzquierdaAdelante, velI);
    digitalWrite(motorIzquierdaAtras, LOW);

    // Motor Derecho
    analogWrite(motorDerechaAdelante, velD);
    digitalWrite(motorDerechaAtras, LOW);
}

// (Aquí irían las funciones de ejecutarCalibracion y cargarCalibracion que ya teníamos)
void ejecutarCalibracion() {
  int blanco[NUM_IR];
  int negro[NUM_IR];

  Serial.println("\n>>> MODO CALIBRACIÓN ACTIVO");
  Serial.println("Poner sobre BLANCO. Lectura en 4 seg...");
  delay(4000);
  for (int i = 0; i < NUM_IR; i++) {
    long suma = 0;
    for (int x = 0; x < 50; x++) suma += analogRead(ir_pins[i]);
    blanco[i] = suma / 50;
    Serial.printf("S%d Blanco: %d\n", i, blanco[i]);
  }

  Serial.println("\nPoner sobre NEGRO (Línea). Lectura en 4 seg...");
  delay(4000);
  for (int i = 0; i < NUM_IR; i++) {
    long suma = 0;
    for (int x = 0; x < 50; x++) suma += analogRead(ir_pins[i]);
    negro[i] = suma / 50;
    negro_calib[i] = negro[i]; // Guardamos para el filtro de aire
    Serial.printf("S%d Negro: %d\n", i, negro[i]);
  }

  Serial.println("\nGuardando datos en memoria...");
  for (int i = 0; i < NUM_IR; i++) {
    umbrales[i] = (blanco[i] + negro[i]) / 2;
    
    String c_umb = "u" + String(i);
    String c_neg = "n" + String(i);
    pref.putInt(c_umb.c_str(), umbrales[i]);
    pref.putInt(c_neg.c_str(), negro_calib[i]);
    
    Serial.printf("S%d Umbral: %d | Negro: %d\n", i, umbrales[i], negro_calib[i]);
  }
  Serial.println("¡CALIBRACIÓN LISTA! Cambia MODO_CALIBRACION a false y resube.");
}

void cargarCalibracion() {
  Serial.println("\n>>> MODO NORMAL: Cargando memoria...");
  for (int i = 0; i < NUM_IR; i++) {
    String c_umb = "u" + String(i);
    String c_neg = "n" + String(i);
    umbrales[i] = pref.getInt(c_umb.c_str(), 2000);
    negro_calib[i] = pref.getInt(c_neg.c_str(), 3500);
    Serial.printf("S%d: U=%d N=%d | ", i, umbrales[i], negro_calib[i]);
  }
  Serial.println("\nListo para correr.");
}