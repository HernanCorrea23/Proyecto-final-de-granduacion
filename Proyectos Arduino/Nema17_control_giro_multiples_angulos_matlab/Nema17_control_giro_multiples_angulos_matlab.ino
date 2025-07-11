#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h"

// --- Constantes ---
const float RAW_A_GRADOS = 360.0 / 4096.0;
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7;

const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 16;
const float RELACION_REDUCTOR = 37.0;
const float PASOS_POR_REV_SALIDA = (PASOS_POR_REV_MOTOR * MICROPASOS) * RELACION_REDUCTOR;
const float PASOS_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;
const int MAX_PUNTOS_LOG = 500;

const float VELOCIDAD_MAXIMA = 1000.0 * MICROPASOS;
const float ACELERACION = 600.0 * MICROPASOS;

// --- Objetos ---
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder; // El encoder ahora se usa solo para verificación

// --- Variables Globales ---
// Ya no necesitamos las variables de seguimiento del encoder para el control
bool homingCompletado = false;

// Variables para el logging
float log_angulos[MAX_PUNTOS_LOG];
unsigned long log_tiempo[MAX_PUNTOS_LOG];
int puntos_log_actuales = 0;
bool registrando_datos = false;
unsigned long tiempo_inicio_ensayo = 0;

// ==========================================================
// --- FUNCIONES AUXILIARES ---
// ==========================================================

void iniciarRegistro() {
  puntos_log_actuales = 0;
  registrando_datos = true;
  tiempo_inicio_ensayo = millis();
  Serial1.println("OK;Registro iniciado.");
}

void finalizarYEnviarRegistro() {
  registrando_datos = false;
  Serial1.println("--- INICIO_DATOS ---");
  for (int i = 0; i < puntos_log_actuales; i++) {
    Serial1.print(log_tiempo[i]);
    Serial1.print(",");
    Serial1.println(log_angulos[i], 4);
  }
  Serial1.println("--- FIN_DATOS ---");
  puntos_log_actuales = 0;
}

// Mueve el motor a una POSICIÓN ABSOLUTA en pasos y espera a que termine.
void moverA_PasosAbsolutos(long pasos_objetivo) {
  motorPasoAPaso.moveTo(pasos_objetivo); // Usamos moveTo para posiciones absolutas

  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();
  }

  // Si estamos registrando, guardamos un punto de datos al finalizar el movimiento
  if (registrando_datos && (puntos_log_actuales < MAX_PUNTOS_LOG)) {
    // Calculamos el ángulo real desde la posición en pasos de la librería
    float angulo_real = (float)motorPasoAPaso.currentPosition() / PASOS_POR_GRADO_SALIDA;
    log_angulos[puntos_log_actuales] = angulo_real;
    log_tiempo[puntos_log_actuales] = millis() - tiempo_inicio_ensayo;
    puntos_log_actuales++;
  }
}

// ==========================================================
// --- SETUP ---
// ==========================================================
void setup() {
  Serial1.begin(115200);
  Wire.begin();
  
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  
  digitalWrite(pinHabilitar, HIGH);

  encoder.begin();
  if (!encoder.isConnected()) {
    Serial1.println("ERROR: Encoder no detectado!");
    while (1);
  }
  
  Serial1.println("--- MODO CALIBRACION MANUAL ---");
  Serial1.println("Alinee manualmente y envie 'h' para establecer el Home.");
}

// ==========================================================
// --- LOOP PRINCIPAL ---
// ==========================================================
void loop() {
  // --- FASE 1: ESPERANDO CALIBRACIÓN DE HOME ---
  if (!homingCompletado) {
    if (Serial1.available() > 0) {
      char comando = Serial1.read();
      if (comando == 'h' || comando == 'H') {
        Serial1.println("\nComando 'h' recibido. Estableciendo Home...");
        
        motorPasoAPaso.setCurrentPosition(0); // Definimos la posición actual como CERO
        digitalWrite(pinHabilitar, LOW);
        
        Serial1.println("--- MODO OPERACION NORMAL ---");
        motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
        motorPasoAPaso.setAcceleration(ACELERACION);
        homingCompletado = true;
      }
    }
    return;
  }

  // --- FASE 2: PROCESAMIENTO DE COMANDOS ---
  if (Serial1.available() > 0) {
    String comando_completo = Serial1.readStringUntil('\n');
    comando_completo.trim();

    if (comando_completo.startsWith("M")) {
      float angulo_obj_absoluto = comando_completo.substring(1).toFloat();
      Serial1.print("OK;Moviendo a angulo absoluto: "); Serial1.println(angulo_obj_absoluto);
      
      // Convertimos el ángulo deseado a una posición absoluta en pasos
      long pasos_objetivo = round(angulo_obj_absoluto * PASOS_POR_GRADO_SALIDA);
      
      moverA_PasosAbsolutos(pasos_objetivo);
      Serial1.println("OK;Movimiento completado.");
    }
    else if (comando_completo.equalsIgnoreCase("START")) {
      iniciarRegistro();
    }
    else if (comando_completo.equalsIgnoreCase("GETDATA")) {
      finalizarYEnviarRegistro();
    }
    else if (comando_completo.equalsIgnoreCase("HOME")) {
        Serial1.println("OK;Volviendo a posicion Home (0)...");
        // La orden es simple: ve a la posición absoluta CERO.
        moverA_PasosAbsolutos(0);
        Serial1.println("OK;En posicion Home.");
    }
  }
}