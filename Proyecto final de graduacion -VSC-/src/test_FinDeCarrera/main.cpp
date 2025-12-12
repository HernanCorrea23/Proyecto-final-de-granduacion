#include <Arduino.h>

/**
 * PROYECTO: Brazo Robótico 2-DOF - Control Manual con Zonas
 * HARDWARE: STM32F103 + DRV8825 + Reductor 39:1
 * LÓGICA: 
 * 0   - 200: BLANCO (Velocidad Normal)
 * 200 - 600: GRIS   (Desaceleración / Aproximación)
 * 600 - 1000: NEGRO  (Parada de Emergencia / Home)
 */

// --- PINOUT ---
#define PIN_SENSOR_ANALOG PA0
#define PIN_STEP          PA5
#define PIN_DIR           PA6

// --- DEFINICIÓN DE ZONAS (Calibración) ---
const int LIMITE_BLANCO = 200;  // Hasta 200 es blanco puro
const int LIMITE_NEGRO  = 600;  // A partir de 600 es negro confirmado

// --- VELOCIDADES (microsegundos de delay entre pasos) ---
// Recuerda: Menor número = Mayor velocidad
const int VEL_CRUCERO = 800;    // Velocidad normal en zona blanca
const int VEL_APROX   = 2500;   // Velocidad lenta en zona gris (precisión)

// Estados del motor
enum EstadoMovimiento { PARADO, MOVIENDO_DERECHA, MOVIENDO_IZQUIERDA };
EstadoMovimiento estado_motor = PARADO;

// Variables globales
int velocidad_actual = VEL_CRUCERO;

void setup() {
  Serial1.begin(115200);
  
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SENSOR_ANALOG, INPUT_ANALOG);

  Serial1.println("--- SISTEMA DE CONTROL POR ZONAS ---");
  Serial1.println("Zonas activas: BLANCO < 200 < GRIS < 600 < NEGRO");
  Serial1.println("Comandos: 'a' (Izq), 'd' (Der), 's' (Stop)");
}

void loop() {
  // 1. Lectura y Diagnóstico del Sensor
  int lectura = analogRead(PIN_SENSOR_ANALOG);
  
  // Imprimir estado solo si hay cambios significativos o cada cierto tiempo
  static long ultima_impresion = 0;
  if (millis() - ultima_impresion > 500) {
    Serial1.print("Sensor: ");
    Serial1.print(lectura);
    Serial1.print(" | Zona: ");
    if (lectura < LIMITE_BLANCO) Serial1.println("BLANCO (Crucero)");
    else if (lectura < LIMITE_NEGRO) Serial1.println("GRIS (Aproximación)");
    else Serial1.println("NEGRO (Stop)");
    ultima_impresion = millis();
  }

  // 2. Gestión de Comandos (Serial)
  if (Serial1.available()) {
    char cmd = Serial1.read();
    if (cmd == 'a') {
      estado_motor = MOVIENDO_IZQUIERDA;
      digitalWrite(PIN_DIR, LOW); 
      Serial1.println("<< IZQUIERDA");
    } 
    else if (cmd == 'd') {
      estado_motor = MOVIENDO_DERECHA;
      digitalWrite(PIN_DIR, HIGH);
      Serial1.println(">> DERECHA");
    }
    else if (cmd == 's') {
      estado_motor = PARADO;
      Serial1.println("|| STOP Manual");
    }
  }

  // 3. Lógica de Control por Zonas
  if (estado_motor != PARADO) {
    
    // CASO A: Zona NEGRA (> 600) -> PARADA TOTAL
    if (lectura >= LIMITE_NEGRO) {
      estado_motor = PARADO;
      Serial1.print("!!! HOME ALCANZADO (Valor: ");
      Serial1.print(lectura);
      Serial1.println(") !!!");
      
      // Retroceso de seguridad (opcional): 
      // Descomentar si el robot se queda "pegado" en el negro
      /*
      digitalWrite(PIN_DIR, !digitalRead(PIN_DIR)); // Invertir
      for(int i=0; i<100; i++) {
        digitalWrite(PIN_STEP, HIGH); delayMicroseconds(10);
        digitalWrite(PIN_STEP, LOW); delayMicroseconds(3000);
      }
      */
    }
    
    // CASO B: Zona GRIS (200 - 600) -> REDUCIR VELOCIDAD
    else if (lectura >= LIMITE_BLANCO) {
      velocidad_actual = VEL_APROX; // Ir más lento
    }
    
    // CASO C: Zona BLANCA (< 200) -> VELOCIDAD NORMAL
    else {
      velocidad_actual = VEL_CRUCERO;
    }

    // 4. Ejecución del paso (Si no estamos parados)
    if (estado_motor != PARADO) {
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_STEP, LOW);
      
      // El delay define la velocidad actual
      delayMicroseconds(velocidad_actual);
    }
  }
}