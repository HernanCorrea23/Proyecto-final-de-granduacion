#include <Arduino.h>
#include <Servo.h>

// --- Definición de Pines ---
// Usamos PA8 porque está libre y soporta PWM
const int pinServo = PA8;

// --- Configuración del Servo ---
Servo miServo;

// --- Ángulos de Calibración ---
// AJUSTAR ESTOS VALORES:
// Los servos baratos a veces no llegan a 0 o 180 grados físicos reales.
// Valores seguros para empezar y no forzar el mecanismo:
int anguloArriba = 90; // Lápiz levantado (no toca el papel)
int anguloAbajo = 130; // Lápiz abajo (toca el papel)

int anguloActual = anguloArriba;
unsigned long tiempoAnterior = 0;
const long intervalo = 1000;

void moverServo(int angulo) {
  // Limitamos el ángulo por seguridad (0 a 180 es el estándar)
  if (angulo < 0)
    angulo = 0;
  if (angulo > 180)
    angulo = 180;

  miServo.write(angulo);
  anguloActual = angulo;

  Serial2.print("Moviendo a: ");
  Serial2.println(anguloActual);
}

void setup() {
  // Iniciamos Serial2 (PA9/PA10) para la comunicación con la PC
  Serial2.begin(115200);
  Serial2.println("\n--- Test de Servo HX5010 para Brazo Robotico ---");
  Serial2.println("Controles:");
  Serial2.println(" 'b' -> Bajar lapiz (Posicion de escritura)");
  Serial2.println(" 's' -> Subir lapiz (Posicion de espera)");
  Serial2.println(" '+' -> Aumentar angulo (+5 grados)");
  Serial2.println(" '-' -> Disminuir angulo (-5 grados)");

  // Inicializar el servo
  miServo.attach(pinServo);

  // Ponerlo en posición segura al inicio (Arriba)
  moverServo(anguloArriba);
}

void loop() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual;
    Serial2.print("Angulo actual: ");
    Serial2.println(anguloActual);
  }

  if (Serial2.available() > 0) {
    char comando = Serial2.read();

    // Convertir a minúscula para facilitar el uso
    comando = tolower(comando);

    switch (comando) {
    case 'b': // Bajar lápiz
      Serial2.println("Comando: BAJAR LAPIZ");
      moverServo(anguloAbajo);
      break;

    case 's': // Subir lápiz
      Serial2.println("Comando: SUBIR LAPIZ");
      moverServo(anguloArriba);
      break;

    case '+': // Ajuste fino +
      moverServo(anguloActual + 5);
      break;

    case '-': // Ajuste fino -
      moverServo(anguloActual - 5);
      break;

    // Ignorar saltos de línea y retornos de carro
    case '\n':
    case '\r':
      break;

    default:
      Serial2.print("Comando no reconocido: ");
      Serial2.println(comando);
      break;
    }
  }
}