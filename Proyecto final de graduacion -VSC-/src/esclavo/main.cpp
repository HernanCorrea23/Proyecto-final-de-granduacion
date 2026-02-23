#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

// --- Configuración Esclavo ---
#define PIN_DE_RE PB12
#define BAUD_RATE 115200

// Pines Motor
#define PIN_STEP PA5
#define PIN_DIR PA6
#define PIN_EN PA7

// --- PIN SEGURO PARA SERVO ---
#define PIN_SERVO PA8

// Mecánica
#define MICROSTEPS 32
#define GEAR_RATIO 39.0
#define MOTOR_STEPS_PER_REV 200.0

// Servo
Servo miServo;
const int anguloArriba = 10;
const int anguloAbajo = 120;
int anguloServoActual = 90;

// Variables de Control de Motor
long lastCmdTime = 0;
const int CMD_TIMEOUT = 200; // ms
int moveState = 0;           // 0: Stop, 1: Up (W), -1: Down (S)
const int STEP_DELAY =
    150; // Microseconds delay between steps (Adjust for speed)

long currentPosition = 0;
long targetPosition = 0;
bool targetMode = false;

long servoDetachTime = 0;
bool isServoAttached = false;

void moverServo(int angulo) {
  if (angulo < 0)
    angulo = 0;
  if (angulo > 180)
    angulo = 180;

  if (!isServoAttached) {
    miServo.attach(PIN_SERVO, 500, 2500);
    isServoAttached = true;
  }
  miServo.write(angulo);

  // Setear el tiempo para desconectar despues (250ms) de este movimiento
  servoDetachTime = millis() + 250;
  anguloServoActual = angulo;
}

void sendResponse(String msg) {
  digitalWrite(PIN_DE_RE, HIGH);
  delayMicroseconds(500);
  Serial1.println(msg);
  Serial1.flush();
  digitalWrite(PIN_DE_RE, LOW);
}

void setup() {
  pinMode(PIN_DE_RE, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PC13, OUTPUT);

  digitalWrite(PIN_DE_RE, LOW);
  digitalWrite(PIN_EN, LOW); // Habilitar driver
  digitalWrite(PC13, HIGH);

  Serial1.begin(BAUD_RATE);
  Serial1.setTimeout(5);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  moverServo(anguloArriba);
}

void loop() {
  // 1. Procesar Comandos
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      char cmd = '\0';
      int valIdx = 1;

      // Buscar el primer caracter válido en la trama (evita basura del RS485 en
      // byte 0)
      for (unsigned int i = 0; i < data.length(); i++) {
        char c = data[i];
        if (c == 'W' || c == 'S' || c == 'D' || c == 'X' || c == 'P' ||
            c == 'L' || c == '+' || c == '-' || c == '?' || c == 'G') {
          cmd = c;
          valIdx = i + 1;
          break;
        }
      }

      if (cmd == 'W') {
        moveState = 1;
        targetMode = false;
        lastCmdTime = millis();
      } else if (cmd == 'S') {
        moveState = -1;
        targetMode = false;
        lastCmdTime = millis();
      } else if (cmd == 'D' || cmd == 'X') { // Stop explícito
        moveState = 0;
        targetMode = false;
      } else if (cmd == 'P') {
        moveState = 0;
        targetMode = false; // Stop motor antes de mover servo por seguridad
        moverServo(anguloArriba);
        sendResponse("SERVO_UP_OK");
      } else if (cmd == 'L') {
        moveState = 0;
        targetMode = false;
        moverServo(anguloAbajo);
        sendResponse("SERVO_DOWN_OK");
      } else if (cmd == '+') {
        moverServo(anguloServoActual + 5);
        sendResponse("SERVO_VAL:" + String(anguloServoActual));
      } else if (cmd == '-') {
        moverServo(anguloServoActual - 5);
        sendResponse("SERVO_VAL:" + String(anguloServoActual));
      } else if (cmd == '?') {
        sendResponse("S_POS:" + String(currentPosition));
      } else if (cmd == 'G') {
        targetPosition = data.substring(valIdx).toInt();
        targetMode = true;
        moveState = 0;
      }
    }
  }

  // 2. Timeout de Seguridad
  if (moveState != 0 && (millis() - lastCmdTime > CMD_TIMEOUT)) {
    moveState = 0;
  }

  // Desconectar servo para evitar ruido, pero de forma no bloqueante
  if (isServoAttached && millis() > servoDetachTime) {
    miServo.detach();
    isServoAttached = false;
  }

  // 3. Ejecutar Movimiento
  if (targetMode) {
    if (currentPosition != targetPosition) {
      if (currentPosition < targetPosition) {
        digitalWrite(PIN_DIR, HIGH); // W was 1 -> HIGH
        currentPosition++;
      } else {
        digitalWrite(PIN_DIR, LOW);
        currentPosition--;
      }
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(5);
      digitalWrite(PIN_STEP, LOW);
      delayMicroseconds(STEP_DELAY);
    } else {
      targetMode = false;
      sendResponse("SDONE");
    }
  } else if (moveState != 0) {
    // Configurar dirección
    digitalWrite(PIN_DIR, (moveState == 1) ? HIGH : LOW);

    // Actualizar Posicion
    if (moveState == 1)
      currentPosition++;
    else
      currentPosition--;

    // Dar un paso
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}
