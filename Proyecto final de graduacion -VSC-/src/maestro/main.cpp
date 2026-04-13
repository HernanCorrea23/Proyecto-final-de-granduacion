#include <AS5600.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

// --- Configuración Maestro (Blue Pill) ---
// RS-485 (Serial1: PA9/PA10, Control: PB12)
// PC (Serial2: PA2/PA3)
// Motor Local (PA5, PA6, PA7)
// Sensor Fin de Carrera (PA0)

// Pines RS-485
#define PIN_DE_RE PB12
#define BAUD_RS485 115200
#define BAUD_PC 115200

// Pines Motor Local (Maestro)
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7;

// Pin Sensor
#define PIN_SENSOR_ANALOG PA0

// Parámetros Motor Local
const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 32;
const float RELACION_REDUCTOR = 39.0;

const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA =
    PASOS_POR_REV_MOTOR_EFECTIVO * RELACION_REDUCTOR;

// Velocidades para AccelStepper (Operación Normal)
const float VELOCIDAD_MAXIMA = 800 * MICROPASOS;
const float VELOCIDAD_MANUAL = 400 * MICROPASOS; // Velocidad para joystick
const float ACELERACION = 300 * MICROPASOS;

// --- PARAMETROS HOMING (Del Test) ---
const int LIMITE_BLANCO = 200;
const int LIMITE_NEGRO = 600;
const int VEL_CRUCERO_HOMING = 1500; // Delay micros
const int VEL_APROX_HOMING = 2500;   // Delay micros
const long AMPLITUD_INICIAL = 1000;
const long RANGO_OSCILACION_FINA = 400;

// Objetos
AccelStepper motorMaestro(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoderMaestro;

// Variables Globales Maestro
bool homingCompletado = false;
bool homingIniciado = false;

// Variables Algoritmo Homing
long posicion_relativa_homing = 0;
long limite_actual_busqueda = AMPLITUD_INICIAL;
bool buscando_negro = true;
bool fase_aproximacion = true;
int estado_motor_homing = 0; // 0: Parado, 1: Derecha, -1: Izquierda
int velocidad_homing_actual = VEL_CRUCERO_HOMING;

// Variables Control Joystick
unsigned long lastInputTime = 0;
const int INPUT_TIMEOUT = 150; // ms
enum Action {
  ACCION_NADA,
  MAESTRO_DER,
  MAESTRO_IZQ,
  ESCLAVO_SUBIR,
  ESCLAVO_BAJAR
};
Action currentAction = ACCION_NADA;
bool slaveMoving = false; // Para enviar STOP al esclavo solo una vez
unsigned long lastSlaveTxTime = 0;
const int SLAVE_TX_INTERVAL = 50; // ms

bool runToTarget = false;

// --- Funciones RS485 ---
void sendRS485(String cmd) {
  digitalWrite(PIN_DE_RE, HIGH);
  delayMicroseconds(500);
  Serial1.print(cmd);
  Serial1.flush();
  digitalWrite(PIN_DE_RE, LOW);
}

void setup() {
  // Comunicación
  Serial1.begin(BAUD_RS485);
  Serial1.setTimeout(5);
  Serial2.begin(BAUD_PC);
  Serial2.setTimeout(5);

  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  // Motor Maestro
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  digitalWrite(pinHabilitar,
               HIGH); // Inicia deshabilitado (se habilitará al empezar)

  // Sensor
  pinMode(PIN_SENSOR_ANALOG, INPUT_ANALOG);

  // Encoder (se mantiene por si se usa en el futuro, aunque no es critico para
  // este homing)
  Wire.begin();
  encoderMaestro.begin();
  encoderMaestro.setDirection(AS5600_CLOCK_WISE);

  delay(2000);
  Serial2.println("\n--- Maestro Iniciado V5 (Auto-Homing + Joystick) ---");
  Serial2.println("ESPERANDO COMANDO:");
  Serial2.println(" 'h' : Iniciar Auto-Homing");
}

// --- LOGICA HOMING (Copiada y adaptada de Test) ---
void loopHoming() {
  // Lectura Sensor
  int lectura = analogRead(PIN_SENSOR_ANALOG);

  // Logica de Movimiento
  if (estado_motor_homing != 0) {
    // A. Verificar Objetivo
    bool objetivo_encontrado = false;
    if (buscando_negro && lectura >= LIMITE_NEGRO)
      objetivo_encontrado = true;
    if (!buscando_negro && lectura <= LIMITE_BLANCO)
      objetivo_encontrado = true;

    if (objetivo_encontrado) {
      if (fase_aproximacion) {
        fase_aproximacion = false;
        posicion_relativa_homing = 0;
        limite_actual_busqueda = RANGO_OSCILACION_FINA;
        buscando_negro = !buscando_negro; // Invertir busqueda

        // Invertir dirección física
        if (estado_motor_homing == 1) { // Estaba Derecha
          estado_motor_homing = -1;
          digitalWrite(pinDireccion, LOW);
        } else { // Estaba Izquierda
          estado_motor_homing = 1;
          digitalWrite(pinDireccion, HIGH);
        }
        Serial2.println(">>> TRANSICION DETECTADA -> Iniciando Barrido Local");
      } else {
        // Fin del homing (segunda confirmacion)
        estado_motor_homing = 0;
        homingCompletado = true;
        homingIniciado = false;

        // Reset AccelStepper
        motorMaestro.setCurrentPosition(0);
        motorMaestro.setMaxSpeed(VELOCIDAD_MAXIMA);
        motorMaestro.setAcceleration(ACELERACION);

        Serial2.println("--- HOME MAESTRO FIJADO (Auto) ---");
        Serial2.println("Controles:");
        Serial2.println(" Maestro: 'd' (Der), 'a' (Izq) [Mantener]");
        Serial2.println(" Esclavo: 'w' (Subir), 's' (Bajar) [Mantener]");
        return;
      }
    }

    // B. Ajustar Velocidad
    if (lectura >= LIMITE_BLANCO && lectura < LIMITE_NEGRO) {
      velocidad_homing_actual = VEL_APROX_HOMING;
    } else {
      velocidad_homing_actual = VEL_CRUCERO_HOMING;
    }

    // C. Oscilacion (Fase Fina)
    if (!fase_aproximacion) {
      if (estado_motor_homing == 1 &&
          posicion_relativa_homing >= limite_actual_busqueda) {
        estado_motor_homing = -1;
        digitalWrite(pinDireccion, LOW);
      } else if (estado_motor_homing == -1 &&
                 posicion_relativa_homing <= -limite_actual_busqueda) {
        estado_motor_homing = 1;
        digitalWrite(pinDireccion, HIGH);
        limite_actual_busqueda += 500; // Expansion failsafe
      }
    }

    // D. Paso Fisico
    digitalWrite(pinPaso, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinPaso, LOW);
    delayMicroseconds(velocidad_homing_actual);

    // E. Actualizar Posicion Relativa
    if (estado_motor_homing == 1)
      posicion_relativa_homing++;
    else if (estado_motor_homing == -1)
      posicion_relativa_homing--;
  }
}

void loop() {
  // --- FASE 1: Homing ---
  if (!homingCompletado) {
    // Escuchar inicio 'h'
    if (Serial2.available()) {
      char cmd = tolower(Serial2.read());
      if (cmd == 'h' && !homingIniciado) {
        homingIniciado = true;
        posicion_relativa_homing = 0;
        fase_aproximacion = true;
        digitalWrite(pinHabilitar, LOW); // Habilitar motor

        int lectura = analogRead(PIN_SENSOR_ANALOG);
        if (lectura < 400) { // Blanco -> Buscar Negro (Izq)
          buscando_negro = true;
          estado_motor_homing = -1;
          digitalWrite(pinDireccion, LOW);
          Serial2.println("Homing: Blanco -> Buscando NEGRO (Izq)");
        } else { // Negro -> Buscar Blanco (Der)
          buscando_negro = false;
          estado_motor_homing = 1;
          digitalWrite(pinDireccion, HIGH);
          Serial2.println("Homing: Negro -> Buscando BLANCO (Der)");
        }
      }
    }

    if (homingIniciado) {
      loopHoming();
    }
    return;
  }

  // --- FASE 2: Operación Normal (Joystick) ---

  // 1. Leer Entrada PC
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      char key = tolower(data[0]);

      // Joystick Maestro
      if (key == 'd' && data.length() == 1) {
        currentAction = MAESTRO_DER;
        lastInputTime = millis();
        runToTarget = false;
      } else if (key == 'a' && data.length() == 1) {
        currentAction = MAESTRO_IZQ;
        lastInputTime = millis();
        runToTarget = false;
      }
      // Joystick Esclavo
      else if (key == 'w' && data.length() == 1) {
        currentAction = ESCLAVO_SUBIR;
        lastInputTime = millis();
        runToTarget = false;
      } else if (key == 's' && data.length() == 1) {
        currentAction = ESCLAVO_BAJAR;
        lastInputTime = millis();
        runToTarget = false;
      }
      // Comandos Servo (Discretos)
      else if (key == 'p') {
        sendRS485("P\n");
        Serial2.println("Servo UP");
      } else if (key == 'l') {
        sendRS485("L\n");
        Serial2.println("Servo DOWN");
      } else if (key == '+') {
        sendRS485("+\n");
      } else if (key == '-') {
        sendRS485("-\n");
      }
      // Comandos Dibujo Absoluto
      else if (key == 'x' && data.length() == 1) {
        currentAction = ACCION_NADA;
        runToTarget = false;
        motorMaestro.stop();
        sendRS485("X\n");
      } else if (key == 'c' && data.length() == 1) {
        Serial2.print("M_POS:");
        Serial2.println(motorMaestro.currentPosition());
        sendRS485("?\n");
      } else if (key == 'g') {
        long target = data.substring(1).toInt();
        motorMaestro.moveTo(target);
        runToTarget = true;
        currentAction = ACCION_NADA;
      } else if (key == 'e') {
        long target = data.substring(1).toInt();
        sendRS485("G" + String(target) + "\n");
      }
    }
  }

  // 2. Timeout Joystick
  if (currentAction != ACCION_NADA &&
      (millis() - lastInputTime > INPUT_TIMEOUT)) {
    currentAction = ACCION_NADA;
  }

  // 3. Ejecutar Accion
  if (runToTarget) {
    if (motorMaestro.distanceToGo() != 0) {
      motorMaestro.run();
    } else {
      runToTarget = false;
      Serial2.println("MDONE");
    }
  } else {
    switch (currentAction) {
    case MAESTRO_DER:
      motorMaestro.setSpeed(VELOCIDAD_MANUAL);
      motorMaestro.runSpeed();
      break;

    case MAESTRO_IZQ:
      motorMaestro.setSpeed(-VELOCIDAD_MANUAL);
      motorMaestro.runSpeed();
      break;

    case ESCLAVO_SUBIR:
      if (millis() - lastSlaveTxTime > SLAVE_TX_INTERVAL) {
        sendRS485("W\n"); // Enviar "Keep Alive" W
        lastSlaveTxTime = millis();
        slaveMoving = true;
      }
      motorMaestro.stop(); // Asegurar maestro quieto
      break;

    case ESCLAVO_BAJAR:
      if (millis() - lastSlaveTxTime > SLAVE_TX_INTERVAL) {
        sendRS485("S\n"); // Enviar "Keep Alive" S
        lastSlaveTxTime = millis();
        slaveMoving = true;
      }
      motorMaestro.stop();
      break;

    case ACCION_NADA:
      motorMaestro.stop(); // Detener Maestro inmediatamente

      // Detener Esclavo si se estaba moviendo
      if (slaveMoving) {
        sendRS485("X\n");
        Serial2.println("Stop Esclavo");
        slaveMoving = false;
      }
      break;
    }
  }

  // 4. Leer Respuestas Esclavo (Feedback Servo, etc.)
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\n');
    if (response.length() > 0) {
      Serial2.print("[Esclavo] ");
      Serial2.println(response);
    }
  }
}