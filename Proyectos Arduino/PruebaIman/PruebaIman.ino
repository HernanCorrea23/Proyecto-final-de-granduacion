#include <Wire.h>
#include <AS5600.h> // O AS5600L, la que te funcionó

AS5600 encoder; // O AS5600L encoder;

float ultimoAnguloMostrado = -1000.0; // Un valor inicial que no sea un ángulo válido
const float UMBRAL_CAMBIO_ANGULO = 0.5; // Umbral en grados para considerar un cambio significativo

void setup() {
  Serial1.begin(115200);
  Wire.begin();
  encoder.begin();
  // encoder.setDirection(AS5600_CLOCK_WISE); // O la que necesites

  bool encoderConectado = encoder.isConnected(); // O tu método de verificación
  /* // Alternativa si isConnected() no existe o falla:
  float anguloPrueba = (float)encoder.rawAngle() * (360.0 / 4096.0);
  if (anguloPrueba >= 0.0 && anguloPrueba <= 360.0) {
      encoderConectado = true;
  } else {
      encoderConectado = false;
  }
  */

  if (!encoderConectado) {
    Serial1.println("Error Encoder en Sketch Simple");
    while (1);
  }
  Serial1.println("Encoder OK en Sketch Simple.");
  Serial1.println("Mueve el eje manualmente para ver nuevas lecturas.");
  Serial1.println("-------------------------------------------------");

  // Mostrar la posición inicial una vez
  mostrarPosicionActual();
}

void loop() {
  // Lee el ángulo actual
  word raw_angle_actual = encoder.rawAngle();
  float grados_actuales = (float)raw_angle_actual * (360.0 / 4096.0);

  // Comprueba si ha habido un cambio significativo desde la última vez que se mostró
  if (abs(grados_actuales - ultimoAnguloMostrado) > UMBRAL_CAMBIO_ANGULO || 
      (abs(grados_actuales - ultimoAnguloMostrado) > 350 && UMBRAL_CAMBIO_ANGULO < 10) ) { // Manejar el cruce por 0/360
    mostrarPosicionActual();
  }
  
  delay(50); // Pequeña pausa para no saturar el bus I2C, pero más rápido que antes
}

// Función para leer y mostrar la posición actual
void mostrarPosicionActual() {
  word raw_angle = encoder.rawAngle();
  float degrees = (float)raw_angle * (360.0 / 4096.0);

  Serial1.print("Raw: ");
  Serial1.print(raw_angle);
  Serial1.print("\t Grados: ");
  Serial1.println(degrees);

  ultimoAnguloMostrado = degrees; // Actualiza el último ángulo mostrado
}