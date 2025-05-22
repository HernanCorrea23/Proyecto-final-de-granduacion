// const int dirPin = PA6;  // Pin para dirección del motor
// const int stepPin = PA5;  // Pin para paso del motor

// const int stepsPerRevolution = 400;  // Número de pasos por revolución (ajustar según el motor) con 200 da una vuelta
// const int speed = 2000;  // Velocidad moderada (ajustar según la velocidad deseada)

// void setup() {
//   pinMode(dirPin, OUTPUT);
//   pinMode(stepPin, OUTPUT);
// }

// void loop() {
//   // Vuelta completa en sentido horario
//   digitalWrite(dirPin, LOW);
//   for (int i = 0; i < stepsPerRevolution; i++) {
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(speed);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(speed);
//   }

//   // Vuelta completa en sentido antihorario
//   digitalWrite(dirPin, HIGH);
//   for (int i = 0; i < stepsPerRevolution; i++) {
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(speed);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(speed);
//   }
// }

// --- Pines Stepper (A4988) ---
const int pinPaso = PA5;
const int pinDireccion = PA6;
// const int pinHabilitar = PAX; // Descomenta y define si necesitas controlar ENABLE

void setup() {
  Serial1.begin(115200); // Para ver mensajes si los añades
  Serial1.println("Iniciando Prueba de Giro Basico del Motor...");

  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  // if (pinHabilitar definido) pinMode(pinHabilitar, OUTPUT);
  // if (pinHabilitar definido) digitalWrite(pinHabilitar, LOW); // Habilitar driver A4988

  // Establecer una dirección de giro (prueba con HIGH o LOW)
  digitalWrite(pinDireccion, HIGH); // Por ejemplo, para girar en un sentido
  Serial1.println("Driver configurado. Enviando pulsos...");
}

// --- Parámetros de Movimiento Básico ---
// Empieza con un retardo GRANDE para una velocidad MUY LENTA.
// Un retardo de 2000us entre flancos de subida y bajada significa
// un periodo de pulso de 4000us (4ms), lo que equivale a 250 pulsos/seg.
// Si estás en micropasos de 1/16, esto es lento. Si estás en paso completo, también es lento.
unsigned long retardoPulso_uS = 50000; // Microsegundos para medio ciclo del pulso STEP

void loop() {
  digitalWrite(pinPaso, HIGH);
  delayMicroseconds(retardoPulso_uS); // Tiempo que el pulso está en ALTO
  digitalWrite(pinPaso, LOW);
  delayMicroseconds(retardoPulso_uS); // Tiempo que el pulso está en BAJO (completa el ciclo)

  // No hay más lógica, solo genera pulsos continuamente.
  // El motor debería girar indefinidamente en la dirección establecida.
}