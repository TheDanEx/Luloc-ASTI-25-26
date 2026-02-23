#include <Preferences.h>

// --- CONFIGURACIÓN DE USUARIO ---
bool MODO_CALIBRACION = false; // Cambia a 'false' para usar los valores guardados
const int NUM_IR = 8;
const int PIN_ENABLE = 33;    // Pin que activa el sensor

// Pines actualizados a tu ESP32 DevKit
const int ir_pins[NUM_IR] = {25, 27, 14, 12, 13, 15, 2, 4};

// Estructuras de datos
Preferences pref;
int umbrales[NUM_IR];      
int negro_calib[NUM_IR];   // Guardamos los valores de negro para el filtro de aire
int ir_values[NUM_IR];
bool binary[NUM_IR];

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH); 
  analogReadResolution(12);       
  
  for (int i = 0; i < NUM_IR; i++) {
    pinMode(ir_pins[i], INPUT);
  }

  pref.begin("robot_asti", false);

  if (MODO_CALIBRACION) {
    ejecutarCalibracion();
  } else {
    cargarCalibracion();
  }
}

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

void loop() {
  for (int i = 0; i < NUM_IR; i++) {
    ir_values[i] = analogRead(ir_pins[i]);
    
    // --- LÓGICA CON MARGEN DE SEGURIDAD (ANTI-BACHES) ---
    
    // 1. Filtro de Aire/Altura: Si el valor es mucho más alto que el negro calibrado
    // significa que no hay suelo o el robot ha saltado.
    int limiteAire = negro_calib[i] * 1.15; 
    if (limiteAire > 4050) limiteAire = 4050; 

    // 2. Histéresis: Añadimos un margen de 250 puntos al umbral.
    // El sensor debe estar "muy seguro" de que ve negro para activarse.
    int umbralConMargen = umbrales[i] + 250;

    if (ir_values[i] > limiteAire) {
      binary[i] = 0; // Demasiado alto/aire -> Blanco
    } 
    else if (ir_values[i] > umbralConMargen) {
      binary[i] = 1; // Negro confirmado
    } 
    else {
      binary[i] = 0; // Blanco/Suelo normal
    }
  }

  // Visualización gráfica
  Serial.print("Binario: ");
  for (int i = 0; i < NUM_IR; i++) {
    Serial.print(binary[i] ? "█" : "░"); 
    Serial.print(" ");
  }
  Serial.print(" | S8 Raw: "); 
  Serial.println(ir_values[7]); 
  
  delay(30); 
}