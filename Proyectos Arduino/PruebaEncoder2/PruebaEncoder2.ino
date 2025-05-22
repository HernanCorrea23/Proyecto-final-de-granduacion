#include <Wire.h>
#include "AS5600.h" // Asegúrate de que este sea el nombre correcto de tu librería
                   // Podría ser también <AS5600L.h> o similar dependiendo de la que uses

// Si tu librería usa una clase específica como AS5600L, cámbiala aquí:
AS5600L encoder; // O AS5600 encoder; si es la clase estándar
// AS5600 encoder; // Descomenta esta y comenta la de arriba si usas la clase AS5600

void setup() {
  Serial1.begin(115200); // Usamos Serial1 para la Blue Pill (PA9=TX, PA10=RX)
  while (!Serial1) {
    ; // Esperar a que el puerto serie se conecte. Necesario para algunos micros.
  }

  Serial1.println("--- Prueba Encoder AS5600 con Blue Pill (STM32) ---");

  // Inicializar I2C. En Blue Pill, los pines por defecto son PB7 (SDA) y PB6 (SCL)
  Wire.begin();
  Serial1.println("I2C Wire iniciado.");

  // Inicializar el encoder
  // La forma exacta puede variar según la librería.
  // Si tu librería requería un pin en begin() (ej. begin(4)), prueba a añadirlo.
  // Lo más común es solo begin() o verificar si está conectado.
  encoder.begin(); // O encoder.begin(PIN_DIR_SI_ES_NECESARIO);
  Serial1.println("Intentando encoder.begin()...");

  // Verificar conexión (el método puede variar según la librería)
  bool conectado = false;
  if (encoder.isConnected()) { // Algunas librerías tienen este método
      conectado = true;
  } else {
      // Si isConnected() no existe o falla, intenta leer un ángulo como prueba
      float anguloTest = encoder.readAngle();
      // Una lectura válida suele ser > -1. Algunas librerías devuelven -1 o valores grandes negativos en error.
      if (anguloTest >= 0 && anguloTest <= 360) { // O simplemente anguloTest != -1 si así lo indica tu librería
          conectado = true;
          Serial1.print("Lectura de ángulo inicial para prueba de conexión: ");
          Serial1.println(anguloTest);
      }
  }


  if (conectado) {
    Serial1.println("¡EXITO! Encoder AS5600 parece conectado e inicializado.");
    // Opcional: Configurar la dirección si es necesario para tu montaje
    // encoder.setDirection(AS5600_CLOCK_WISE); // O AS5600_COUNTER_CLOCK_WISE
    // Serial1.print("Dirección de software configurada a: ");
    // Serial1.println(encoder.getDirection());
  } else {
    Serial1.println("¡FALLO! Error al conectar/inicializar el encoder.");
    Serial1.println("Verifica cableado: SDA (PB7), SCL (PB6), VCC (3.3V desde regulador externo recomendado), GND, y resistencias pull-up.");
    while (1); // Detener si falla la inicialización
  }

  Serial1.println("\nComenzando lectura continua de ángulos...");
}

void loop() {
  // Leer el ángulo escalado (usualmente en grados)
  float anguloEnGrados = encoder.readAngle(); // O getAngle(), según tu librería

  // Leer el valor raw del ángulo (usualmente 0-4095 para 12 bits)
  word anguloRaw = encoder.rawAngle();       // O getRawAngle(), según tu librería

  Serial1.print("Angulo (grados): ");
  Serial1.print(anguloEnGrados, 2); // Imprime con 2 decimales
  Serial1.print("\t Angulo Raw: ");
  Serial1.print(anguloRaw);

  // Opcional: Verificar estado del imán si tu librería lo soporta
  // byte estadoMagnet = encoder.detectMagnet(); // La función y los valores de retorno varían
  // Serial1.print("\t Estado Magnet: ");
  // Serial1.print(estadoMagnet); // Interpreta el valor (MD, ML, MH)

  Serial1.println();

  delay(200); // Espera 200 milisegundos entre lecturas
}