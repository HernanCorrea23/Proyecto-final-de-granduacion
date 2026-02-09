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
#define PIN_SERVO PB0 
// Mecánica
#define MICROSTEPS 32
#define GEAR_RATIO 39.0
#define MOTOR_STEPS_PER_REV 200.0
const float STEPS_PER_DEGREE = (MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;
Servo miServo;
const int anguloArriba = 90;  
const int anguloAbajo = 130; 
int anguloServoActual = 90;
void moverServo(int angulo) {
  if (angulo < 0) angulo = 0;
  if (angulo > 180) angulo = 180;
  miServo.write(angulo);
  anguloServoActual = angulo;
  delay(20); 
}
void setup() {
  pinMode(PIN_DE_RE, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PC13, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);
  digitalWrite(PIN_EN, LOW); 
  digitalWrite(PC13, HIGH);
  
  Serial1.begin(BAUD_RATE);
  // Aseguramos pines I2C para evitar bloqueos
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  
  // Attach en el nuevo pin seguro PB0
  miServo.attach(PIN_SERVO, 500, 2500); 
  moverServo(anguloArriba);
}
void doSteps(long steps, bool dir) {
  digitalWrite(PIN_DIR, dir ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    if (Serial1.available()) {
        char c = Serial1.peek();
        if (c == 'D' || c == 'X') { Serial1.read(); break; }
    }
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(150);
  }
}
void sendResponse(String msg) {
  digitalWrite(PIN_DE_RE, HIGH);
  delayMicroseconds(500);
  Serial1.println(msg);
  Serial1.flush();
  digitalWrite(PIN_DE_RE, LOW);
}
void loop() {
  if (Serial1.available()) {
    char cmd = Serial1.read();
    if (cmd == '\n' || cmd == '\r') return;
    if (cmd == 'W') {
      doSteps(2 * STEPS_PER_DEGREE, true);
    } 
    else if (cmd == 'S') {
      doSteps(2 * STEPS_PER_DEGREE, false);
    }
    else if (cmd == 'P') { 
      moverServo(anguloArriba);
      sendResponse("SERVO_UP_OK"); // ESTO DEBE APARECER EN EL MONITOR PC
      digitalWrite(PC13, LOW); delay(50); digitalWrite(PC13, HIGH); 
    }
    else if (cmd == 'L') { 
      moverServo(anguloAbajo);
      sendResponse("SERVO_DOWN_OK"); // ESTO TAMBIÉN
      digitalWrite(PC13, LOW); delay(50); digitalWrite(PC13, HIGH);
    }
    else if (cmd == '+') {
      moverServo(anguloServoActual + 5);
      sendResponse("SERVO_VAL:" + String(anguloServoActual));
    }
    else if (cmd == '-') {
      moverServo(anguloServoActual - 5);
      sendResponse("SERVO_VAL:" + String(anguloServoActual));
    }
    
    while(Serial1.available()) Serial1.read();
  }
}