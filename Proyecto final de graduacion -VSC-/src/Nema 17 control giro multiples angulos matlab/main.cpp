#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h"

// ==========================================================
// --- CONFIGURACIÓN Y CONSTANTES ---
// ==========================================================
const float RAW_A_GRADOS = 360.0 / 4096.0;
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7;

// --- PARÁMETROS MECÁNICOS ---
const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 32;                  
const float RELACION_REDUCTOR = 39.0;      
const float BACKLASH_COMPENSATION_GRADOS = 1.5; 

// Cálculos
const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA = PASOS_POR_REV_MOTOR_EFECTIVO * RELACION_REDUCTOR; 
const float PASOS_MOTOR_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

// Velocidad (Segura)
const float VELOCIDAD_MAXIMA = 800.0 * MICROPASOS; 
const float ACELERACION = 300.0 * MICROPASOS; 

// --- LOGGING DE DATOS ---
const int MAX_PUNTOS_LOG = 600; // Ajustado para la RAM de la BluePill
float log_angulos[MAX_PUNTOS_LOG];
unsigned long log_tiempo[MAX_PUNTOS_LOG];
int indice_log = 0;
unsigned long tiempo_inicio_movimiento = 0;

// Objetos
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder;

// Variables Globales
long vueltasCompletas = 0;
long lecturaEncoderAnterior = 0;
long encoderOffset = 0; 
bool homingCompletado = false;
int lastMoveDirection = 0; 

// ==========================================================
// --- FUNCIONES AUXILIARES ---
// ==========================================================

long leerPasosCrudosEncoder() {
  return encoder.readAngle();
}

long leerPosicionContinuaEncoder() {
  long lecturaCrudaActual = leerPasosCrudosEncoder();
  long diferenciaCruda = lecturaCrudaActual - lecturaEncoderAnterior;
  if (diferenciaCruda < -2048) vueltasCompletas++;
  else if (diferenciaCruda > 2048) vueltasCompletas--;
  lecturaEncoderAnterior = lecturaCrudaActual;
  return (vueltasCompletas * 4096) + lecturaCrudaActual;
}

float leerAnguloEncoderEnGrados() {
  long posicionRelativa = leerPosicionContinuaEncoder() - encoderOffset;
  return (float)posicionRelativa * RAW_A_GRADOS;
}

// Función para registrar datos en RAM sin bloquear
void registrarDato() {
  if (indice_log < MAX_PUNTOS_LOG) {
      log_angulos[indice_log] = leerAnguloEncoderEnGrados();
      log_tiempo[indice_log] = millis() - tiempo_inicio_movimiento;
      indice_log++;
  }
}

// Función que mueve el motor y graba datos simultáneamente
void correrStepperHastaPosicion(long posicionObjetivoLogico, bool grabar) {
  motorPasoAPaso.moveTo(posicionObjetivoLogico);
  unsigned long ultimaLectura = 0;
  unsigned long ultimoLog = 0;

  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();
    
    unsigned long ahora = millis();

    // 1. Actualizar Encoder (rápido, para no perder vueltas)
    if (ahora - ultimaLectura >= 2) {
       leerPosicionContinuaEncoder(); 
       ultimaLectura = ahora;
    }

    // 2. Grabar Datos (cada 15ms para no llenar la RAM muy rápido)
    if (grabar && (ahora - ultimoLog >= 15)) {
       registrarDato();
       ultimoLog = ahora;
    }
  }
  delay(100); 
  leerPosicionContinuaEncoder(); // Lectura final
  if (grabar) registrarDato(); // Dato final
}

void moverConCompensacion(long posicionLogicaObjetivo, bool grabar) {
    long backlash_steps = round(BACKLASH_COMPENSATION_GRADOS * PASOS_MOTOR_POR_GRADO_SALIDA);
    int currentDirection = 0;

    if (posicionLogicaObjetivo > motorPasoAPaso.currentPosition()) currentDirection = 1;
    else if (posicionLogicaObjetivo < motorPasoAPaso.currentPosition()) currentDirection = -1;

    // Compensación de Backlash
    if (currentDirection != 0 && lastMoveDirection != 0 && currentDirection != lastMoveDirection) {
      long posicionCompensada = posicionLogicaObjetivo + (currentDirection * backlash_steps);
      // Movemos un poco más allá (sin grabar datos de este ajuste mecánico)
      correrStepperHastaPosicion(posicionCompensada, false); 
      delay(50);
    }

    // Movimiento real (aquí sí grabamos si se pide)
    correrStepperHastaPosicion(posicionLogicaObjetivo, grabar);
    
    if (currentDirection != 0) lastMoveDirection = currentDirection;
}

// Envía todos los datos acumulados a MATLAB
void enviarDatos() {
  Serial1.println("---INICIO---");
  for(int i=0; i<indice_log; i++) {
    Serial1.print(log_tiempo[i]);
    Serial1.print(",");
    Serial1.println(log_angulos[i], 2);
  }
  Serial1.println("---FIN---");
  indice_log = 0; // Resetear buffer
}

// ==========================================================
// --- SETUP ---
// ==========================================================
void setup() {
  Serial1.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  delay(100); 

  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  digitalWrite(pinHabilitar, HIGH); 

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);
  lecturaEncoderAnterior = leerPasosCrudosEncoder(); // Inicializar

  // Chequeo rápido
  if (!encoder.isConnected()) {
     Serial1.println("ERROR: Encoder no conectado");
     while(1);
  }
  Serial1.println("READY"); // Señal para MATLAB
}

// ==========================================================
// --- LOOP PRINCIPAL (PROCESADOR DE COMANDOS) ---
// ==========================================================
void loop() {
  // Mantener el encoder actualizado siempre
  static unsigned long lastKeepAlive = 0;
  if(millis() - lastKeepAlive > 10) {
    leerPosicionContinuaEncoder();
    lastKeepAlive = millis();
  }

  if (Serial1.available() > 0) {
    char comando = Serial1.read();

    // --- COMANDOS MANUALES / HOMING ---
    if (comando == 'j') { // Izquierda
       digitalWrite(pinHabilitar, LOW);
       motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA/2);
       motorPasoAPaso.move(-2000);
       while(motorPasoAPaso.distanceToGo() != 0) {
         motorPasoAPaso.run();
         leerPosicionContinuaEncoder();
       }
    }
    else if (comando == 'k') { // Derecha
       digitalWrite(pinHabilitar, LOW);
       motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA/2);
       motorPasoAPaso.move(2000);
       while(motorPasoAPaso.distanceToGo() != 0) {
         motorPasoAPaso.run();
         leerPosicionContinuaEncoder();
       }
    }
    else if (comando == 'h') { // SET HOME
       motorPasoAPaso.stop();
       encoderOffset = leerPosicionContinuaEncoder();
       motorPasoAPaso.setCurrentPosition(0);
       motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
       motorPasoAPaso.setAcceleration(ACELERACION);
       homingCompletado = true;
       Serial1.println("HOME_OK");
    }
    
    // --- COMANDO DE MOVIMIENTO AUTOMATICO ---
    // Formato: "m90.5" (mueve a 90.5 grados)
    else if (comando == 'm') {
       if(!homingCompletado) {
         Serial1.println("ERROR: Homing requerido");
         return;
       }
       
       float anguloDeseado = Serial1.parseFloat(); // Lee el número que sigue a la 'm'
       
       Serial1.print("MOVIENDO:"); Serial1.println(anguloDeseado);
       
       // 1. Preparar grabación
       indice_log = 0;
       tiempo_inicio_movimiento = millis();
       
       // 2. Calcular pasos y mover
       long pasosObjetivo = round(anguloDeseado * PASOS_MOTOR_POR_GRADO_SALIDA);
       moverConCompensacion(pasosObjetivo, true); // true = grabar datos
       
       // 3. Enviar datos a MATLAB
       enviarDatos();
    }
  }
}