#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// --- Configuración Esclavo ---
#define PIN_DE_RE PB12
#define BAUD_RATE 115200

// Pines Motor
#define PIN_STEP PA5
#define PIN_DIR  PA6
#define PIN_EN   PA7

// --- PIN SEGURO PARA SERVO ---
#define PIN_SERVO PA8

// Mecánica
#define MICROSTEPS 32
#define GEAR_RATIO 39.0
#define MOTOR_STEPS_PER_REV 200.0

// Servo
Servo miServo;
const int anguloArriba = 10;  
const int anguloAbajo = 90;
int anguloServoActual = 90;

// Variables de Control de Motor
long lastCmdTime = 0;
const int CMD_TIMEOUT = 200; // ms
int moveState = 0; // 0: Stop, 1: Up (W), -1: Down (S)
const int STEP_DELAY = 150; // Microseconds delay between steps (Adjust for speed)

void moverServo(int angulo) {
  if (angulo < 0) angulo = 0;
  if (angulo > 180) angulo = 180;
 
  miServo.attach(PIN_SERVO, 500, 2500);
  miServo.write(angulo);
  delay(250);
  miServo.detach();
 
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
  
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
 
  moverServo(anguloArriba);
}

void loop() {
  // 1. Procesar Comandos
  if (Serial1.available()) {
    char cmd = Serial1.read();
    
    // Ignorar caracteres de nueva línea
    if (cmd != '\n' && cmd != '\r') {
      
      if (cmd == 'W') {
        moveState = 1;
        lastCmdTime = millis();
      }
      else if (cmd == 'S') {
        moveState = -1;
        lastCmdTime = millis();
      }
      else if (cmd == 'D' || cmd == 'X') { // Stop explícito
        moveState = 0;
      }
      else if (cmd == 'P') {
        moveState = 0; // Stop motor antes de mover servo por seguridad
        moverServo(anguloArriba);
        sendResponse("SERVO_UP_OK");
      }
      else if (cmd == 'L') {
        moveState = 0;
        moverServo(anguloAbajo);
        sendResponse("SERVO_DOWN_OK");
      }
      else if (cmd == '+') {
        moverServo(anguloServoActual + 5);
        sendResponse("SERVO_VAL:" + String(anguloServoActual));
      }
      else if (cmd == '-') {
        moverServo(anguloServoActual - 5);
        sendResponse("SERVO_VAL:" + String(anguloServoActual));
      }
    }
  }

  // 2. Timeout de Seguridad
  if (moveState != 0 && (millis() - lastCmdTime > CMD_TIMEOUT)) {
    moveState = 0;
  }

  // 3. Ejecutar Movimiento
  if (moveState != 0) {
    // Configurar dirección
    // W (Up) -> true (según código anterior W era true)
    // S (Down) -> false
    digitalWrite(PIN_DIR, (moveState == 1) ? HIGH : LOW);
    
    // Dar un paso
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(STEP_DELAY); 
  }
}

