
#include "AS5600.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

// --- Constantes ---
const float RAW_A_GRADOS = 360.0 / 4096.0;
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7;

// --- PARÁMETROS ---
const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 32;
const float RELACION_REDUCTOR = 39.0;
const float BACKLASH_COMPENSATION_GRADOS = 1.5;

const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA =
    PASOS_POR_REV_MOTOR_EFECTIVO * RELACION_REDUCTOR;
const float PASOS_MOTOR_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

const float VELOCIDAD_MAXIMA =
    600 * MICROPASOS; // Reducido ligeramente para estabilidad
const float ACELERACION = 300 * MICROPASOS;

// --- Objetos ---
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder;

// --- Variables Globales ---
long vueltasCompletas = 0;
long lecturaEncoderAnterior = 0;
long encoderOffset = 0;
bool homingCompletado = false;
int homingDirection = 0; // 0=stop, 1=derecha, -1=izquierda
int lastMoveDirection = 0;

// Variables para serial
String inputString = "";     // String to hold incoming data
bool stringComplete = false; // whether the string is complete
unsigned long lastStatusTime = 0;

// --- Funciones Auxiliares ---

long leerPasosCrudosEncoder() { return encoder.readAngle(); }

long leerPosicionContinuaEncoder() {
  long lecturaCrudaActual = leerPasosCrudosEncoder();
  long diferenciaCruda = lecturaCrudaActual - lecturaEncoderAnterior;
  if (diferenciaCruda < -2048) {
    vueltasCompletas++;
  } else if (diferenciaCruda > 2048) {
    vueltasCompletas--;
  }
  lecturaEncoderAnterior = lecturaCrudaActual;
  return (vueltasCompletas * 4096) + lecturaCrudaActual;
}

float leerAnguloEncoderEnGrados() {
  long posicionRelativa = leerPosicionContinuaEncoder() - encoderOffset;
  return (float)posicionRelativa * RAW_A_GRADOS;
}

// Nueva función optimizada: Permite elegir si leer sensor real o estimar
void reportarPosicion(bool usarEncoderReal) {
  // Enviar datos cada 150ms para no saturar
  if (millis() - lastStatusTime > 150) {
    float anguloReportado;

    if (usarEncoderReal) {
      // Lectura REAL (Lenta, usa I2C blocking) - Usar solo en reposo o Jog
      // lento
      anguloReportado = leerAnguloEncoderEnGrados();
    } else {
      // Lectura ESTIMADA (Instantanea) - Usar durante movimientos rápidos para
      // no causar saltos Calculamos grados basados en los pasos lógicos
      // actuales
      anguloReportado =
          motorPasoAPaso.currentPosition() / PASOS_MOTOR_POR_GRADO_SALIDA;
    }

    Serial1.print("POS:");
    Serial1.println(anguloReportado, 2);
    lastStatusTime = millis();
  }
}

void correrStepperHastaPosicion(long posicionObjetivoLogico) {
  motorPasoAPaso.moveTo(posicionObjetivoLogico);
  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();
    // Durante el movimiento rápido, usamos FALSE para no frenar el motor con
    // lecturas I2C
    reportarPosicion(false);
  }
}

void moverConCompensacion(long posicionLogicaObjetivo) {
  long backlash_steps =
      round(BACKLASH_COMPENSATION_GRADOS * PASOS_MOTOR_POR_GRADO_SALIDA);
  int currentDirection = 0;

  if (posicionLogicaObjetivo > motorPasoAPaso.currentPosition()) {
    currentDirection = 1;
  } else if (posicionLogicaObjetivo < motorPasoAPaso.currentPosition()) {
    currentDirection = -1;
  }

  if (currentDirection != 0 && lastMoveDirection != 0 &&
      currentDirection != lastMoveDirection) {
    long posicionCompensada =
        posicionLogicaObjetivo + (currentDirection * backlash_steps);
    correrStepperHastaPosicion(posicionCompensada);
    delay(100);
  }

  correrStepperHastaPosicion(posicionLogicaObjetivo);
  if (currentDirection != 0) {
    lastMoveDirection = currentDirection;
  }
}

void procesarComando(String comando) {
  comando.trim();
  if (comando.length() == 0)
    return;

  // -- COMANDOS DE HOMING --
  if (comando == "JOG_L") {
    // Jog Left
    if (!homingCompletado) {
      digitalWrite(pinHabilitar, LOW);
      motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA / 4);
      motorPasoAPaso.setAcceleration(ACELERACION / 4);
      motorPasoAPaso.moveTo(-2000000000);
      homingDirection = -1;
      Serial1.println("STATUS:JOGGING_LEFT");
    }
  } else if (comando == "JOG_R") {
    // Jog Right
    if (!homingCompletado) {
      digitalWrite(pinHabilitar, LOW);
      motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA / 4);
      motorPasoAPaso.setAcceleration(ACELERACION / 4);
      motorPasoAPaso.moveTo(2000000000);
      homingDirection = 1;
      Serial1.println("STATUS:JOGGING_RIGHT");
    }
  } else if (comando == "STOP_JOG") {
    if (homingDirection != 0) {
      homingDirection = 0;
      motorPasoAPaso.stop();
      motorPasoAPaso.runToPosition();
      Serial1.println("STATUS:STOPPED");
    }
  } else if (comando == "SET_HOME") {
    homingDirection = 0;
    motorPasoAPaso.stop();
    motorPasoAPaso.runToPosition();

    lecturaEncoderAnterior = leerPasosCrudosEncoder();
    vueltasCompletas = 0;
    encoderOffset = leerPosicionContinuaEncoder();

    motorPasoAPaso.setCurrentPosition(0);
    digitalWrite(pinHabilitar, LOW); // Habilitar

    // Restaurar velocidad normal
    motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
    motorPasoAPaso.setAcceleration(ACELERACION);

    homingCompletado = true;
    Serial1.println("STATUS:HOME_SET");
  }

  // -- COMANDOS DE CONTROL --
  else if (comando.startsWith("GOTO:")) {
    if (homingCompletado) {
      float anguloObjetivo = comando.substring(5).toFloat();
      Serial1.print("STATUS:MOVING_TO:");
      Serial1.println(anguloObjetivo);

      long pasosRelativosMotor =
          round(anguloObjetivo * PASOS_MOTOR_POR_GRADO_SALIDA);

      long posicionLogicaObjetivo = pasosRelativosMotor;

      moverConCompensacion(posicionLogicaObjetivo);

      // Al terminar el movimiento, forzamos un reporte ESTIMADO para mantener
      // la gráfica estable (usuario prefiere target)
      reportarPosicion(false);
      Serial1.println("STATUS:MOVE_COMPLETE");
    } else {
      Serial1.println("ERROR:HOME_NOT_SET");
    }
  } else if (comando == "PING") {
    Serial1.println("PONG");
  }
}

void setup() {
  Serial1.begin(115200); // Usamos Serial1 como en el original
  inputString.reserve(200);

  Wire.begin();
  delay(100);

  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  digitalWrite(pinHabilitar, HIGH); // Deshabilitar motor.

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);

  // Verificación simple de conexión
  if (!encoder.isConnected()) {
    Serial1.println("ERROR:ENCODER_NOT_CONNECTED");
    // Intentar una vez mas
    delay(500);
  }
}

void loop() {
  // 1. Manejo del motor en bucle (para el Jogging continuo)
  if (homingDirection != 0) {
    motorPasoAPaso.run();
    // En JOG vamos lento, evitar I2C blocking para no causar saltos
    reportarPosicion(false);
  }

  // 2. Lectura Serial no bloqueante
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    procesarComando(inputString);
    inputString = "";
    stringComplete = false;
  }

  // 3. Reporte de estado periódico (Posición) cuando está quieto
  reportarPosicion(false); // Reportamos lógica para mantener gráfica estable
                           // (petición usuario)
}
