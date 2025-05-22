#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

const int stepPin = PA5; // Pin "step" conectado al pin A5 del Blue Pill
const int dirPin = PA6;  // Pin "dir" conectado al pin A6 del Blue Pill
// Parámetros para la aceleración
unsigned long delayTime = 1500;  // Delay inicial en microsegundos
const unsigned long minDelayTime = 500;  // Delay mínimo para alcanzar velocidad máxima
const int accelerationSteps = 100; // Número de pasos para acelerar

void setup() {
  Serial1.begin(9600);
  Wire.begin(); // Inicializa la comunicación I2C en los pines por defecto PB6 y PB7

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  if (!encoder.begin()) {
    Serial1.println("Error al inicializar el encoder AS5600");
    while (1);
  }
  Serial1.println("Encoder AS5600 inicializado correctamente");
}

void loop() {
  static int lastPosition = -1;
  int position = encoder.readAngle();

  if (position != lastPosition) {
    Serial1.print("Posición del encoder: ");
    Serial1.println(position);
    lastPosition = position;
  }
  
  // Aceleración simple: reduce el delay cada cierto número de pasos hasta el mínimo
  for (int i = 0; i < accelerationSteps; i++) {
    digitalWrite(dirPin, HIGH); // Dirección
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime); 
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);

    // Reducción progresiva del delay para acelerar
    if (delayTime > minDelayTime) {
      delayTime -= (1500 - minDelayTime) / accelerationSteps;
    }
  }

  // Una vez acelerado, mantener velocidad constante
  for (int i = 0; i < 500; i++) { // Ciclo para mantener la velocidad
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(minDelayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(minDelayTime);
  }
}






