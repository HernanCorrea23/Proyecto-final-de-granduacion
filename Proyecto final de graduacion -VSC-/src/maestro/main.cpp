#include <AS5600.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

// --- Configuración Maestro (Blue Pill) ---
// RS-485 (Serial1: PA9/PA10, Control: PB12)
// PC (Serial2: PA2/PA3)
// Motor Local (PA5, PA6, PA7)
// Encoder Local (I2C: PB6, PB7)

// Pines RS-485
#define PIN_DE_RE PB12
#define BAUD_RS485 115200
#define BAUD_PC 115200

// Pines Motor Local (Maestro)
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7;

// Parámetros Motor Local
const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 32;
const float RELACION_REDUCTOR = 39.0;
const float BACKLASH_COMPENSATION_GRADOS = 1.5;

const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA =
    PASOS_POR_REV_MOTOR_EFECTIVO * RELACION_REDUCTOR;
const float PASOS_MOTOR_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

const float VELOCIDAD_MAXIMA = 800 * MICROPASOS;
const float ACELERACION = 300 * MICROPASOS;
const float TOLERANCIA_ANGULO = 1.0;
const float RAW_A_GRADOS = 360.0 / 4096.0;

// Objetos
AccelStepper motorMaestro(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoderMaestro;

// Variables Globales Maestro
long vueltasCompletas = 0;
long lecturaEncoderAnterior = 0;
long encoderOffset = 0;
bool homingCompletado = false;
int homingDirection = 0;
int lastMoveDirection = 0;
float anguloObjetivoRelativoActual = 0.0;

// --- Funciones Encoder Maestro ---

long leerPasosCrudosEncoder() { return encoderMaestro.readAngle(); }

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

// --- Control RS485 ---
void sendRS485(String cmd) {
  digitalWrite(PIN_DE_RE, HIGH);
  delayMicroseconds(500);
  Serial1.print(cmd);
  Serial1.flush();
  digitalWrite(PIN_DE_RE, LOW);
}

// --- Control Motor Maestro ---

void correrStepperHastaPosicion(long posicionObjetivoLogico) {
  motorMaestro.moveTo(posicionObjetivoLogico);
  while (motorMaestro.distanceToGo() != 0) {
    motorMaestro.run();
  }
  delay(200);
}

void moverConCompensacion(long posicionLogicaObjetivo) {
  long backlash_steps =
      round(BACKLASH_COMPENSATION_GRADOS * PASOS_MOTOR_POR_GRADO_SALIDA);
  int currentDirection = 0;

  if (posicionLogicaObjetivo > motorMaestro.currentPosition()) {
    currentDirection = 1;
  } else if (posicionLogicaObjetivo < motorMaestro.currentPosition()) {
    currentDirection = -1;
  }

  if (currentDirection != 0 && lastMoveDirection != 0 &&
      currentDirection != lastMoveDirection) {
    Serial2.println("Compensando backlash Maestro...");
    long posicionCompensada =
        posicionLogicaObjetivo + (currentDirection * backlash_steps);
    correrStepperHastaPosicion(posicionCompensada);
    delay(100);
  }

  correrStepperHastaPosicion(posicionLogicaObjetivo);
  if (currentDirection != 0)
    lastMoveDirection = currentDirection;
}

void setup() {
  // Comunicación
  Serial1.begin(BAUD_RS485);
  Serial2.begin(BAUD_PC);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  // Motor Maestro
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  digitalWrite(pinHabilitar, HIGH); // Inicia deshabilitado

  // Encoder Maestro
  Wire.begin();
  encoderMaestro.begin();
  encoderMaestro.setDirection(AS5600_CLOCK_WISE);

  delay(2000);
  Serial2.println("\n--- Maestro Iniciado V4 ---");
  Serial2.println("CALIBRACION MOTOR MAESTRO:");
  Serial2.println(" 'j' / 'k' : Mover Izq/Der");
  Serial2.println(" 'h'       : Fijar HOME y empezar");
}

void loop() {
  // --- FASE 1: Homing Maestro ---
  if (!homingCompletado) {
    if (homingDirection != 0) {
      motorMaestro.run();
    }

    if (Serial2.available() > 0) {
      char cmd = tolower(Serial2.read());
      if (cmd == 'j') {
        digitalWrite(pinHabilitar, LOW);
        motorMaestro.setMaxSpeed(VELOCIDAD_MAXIMA / 4);
        motorMaestro.setAcceleration(ACELERACION / 4);
        motorMaestro.moveTo(-2000000000);
        homingDirection = -1;
        Serial2.println("Maestro: Hacia Izquierda...");
      } else if (cmd == 'k') {
        digitalWrite(pinHabilitar, LOW);
        motorMaestro.setMaxSpeed(VELOCIDAD_MAXIMA / 4);
        motorMaestro.setAcceleration(ACELERACION / 4);
        motorMaestro.moveTo(2000000000);
        homingDirection = 1;
        Serial2.println("Maestro: Hacia Derecha...");
      } else if (cmd == 'h') {
        homingDirection = 0;
        motorMaestro.stop();
        motorMaestro.runToPosition();

        lecturaEncoderAnterior = leerPasosCrudosEncoder();
        vueltasCompletas = 0;
        encoderOffset = leerPosicionContinuaEncoder();
        motorMaestro.setCurrentPosition(0);

        motorMaestro.setMaxSpeed(VELOCIDAD_MAXIMA);
        motorMaestro.setAcceleration(ACELERACION);
        homingCompletado = true;

        Serial2.println("--- HOME MAESTRO FIJADO ---");
        Serial2.println(
            "Controles Maestro: 'd':+90, 'i':-90, 'e':+45, 'q':-45");
        Serial2.println(
            "Controles Esclavo: 'w'/'s':motor, 'x':stop, 'p'/'l':servo");
      }
    }
    return;
  }

  // --- FASE 2: Operación Normal ---

  // 1. Escuchar PC (Serial2)
  if (Serial2.available()) {
    char key = Serial2.read();

    // COMANDOS LOCALES (Motor Maestro)
    if (key == 'd') {
      Serial2.println("Maestro -> 90 y Volver");
      moverConCompensacion(round(90.0 * PASOS_MOTOR_POR_GRADO_SALIDA));
      delay(1000);
      moverConCompensacion(0);
      Serial2.println("Maestro en Home (0).");
    } else if (key == 'i') {
      Serial2.println("Maestro -> -90 y Volver");
      moverConCompensacion(round(-90.0 * PASOS_MOTOR_POR_GRADO_SALIDA));
      delay(1000);
      moverConCompensacion(0);
      Serial2.println("Maestro en Home (0).");
    } else if (key == 'e') {
      Serial2.println("Maestro -> 45 y Volver");
      moverConCompensacion(round(45.0 * PASOS_MOTOR_POR_GRADO_SALIDA));
      delay(1000);
      moverConCompensacion(0);
      Serial2.println("Maestro en Home (0).");
    } else if (key == 'q') {
      Serial2.println("Maestro -> -45 y Volver");
      moverConCompensacion(round(-45.0 * PASOS_MOTOR_POR_GRADO_SALIDA));
      delay(1000);
      moverConCompensacion(0);
      Serial2.println("Maestro en Home (0).");
    }

    // COMANDOS REMOTOS (PC -> RS485 -> Esclavo)
    else if (key == 'w' || key == 'W') {
      sendRS485("W");
      Serial2.println("Remoto -> SUBIR");
    } else if (key == 's' || key == 'S') {
      sendRS485("S");
      Serial2.println("Remoto -> BAJAR");
    } else if (key == 'x' || key == 'X') {
      sendRS485("D");
      Serial2.println("Remoto -> !!! STOP !!!");
    } else if (key == 'p' || key == 'P') {
      sendRS485("P");
      Serial2.println("Servo -> UP");
    } else if (key == 'l' || key == 'L') {
      sendRS485("L");
      Serial2.println("Servo -> DOWN");
    } else if (key == '+') {
      sendRS485("+");
      Serial2.println("Servo -> +5");
    } else if (key == '-') {
      sendRS485("-");
      Serial2.println("Servo -> -5");
    }
  }

  // 2. Escuchar respuestas del Esclavo (Serial1)
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\n');
    if (response.length() > 0) {
      Serial2.print("[Esclavo] ");
      Serial2.println(response);
      digitalWrite(PC13, !digitalRead(PC13));
    }
  }

  // 3. Mantener motor local (si no estamos en moverConCompensacion que bloquea)
  motorMaestro.run();
}
