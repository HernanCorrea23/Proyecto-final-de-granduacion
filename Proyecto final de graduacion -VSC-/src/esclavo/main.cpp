#include <Arduino.h>

// Definir el pin de control para el módulo MAX485 (DE/RE)
const int MAX485_DE_RE = PA8;

void setup() {
  // Inicializar el puerto serie (UART2) para monitoreo
  // PA2 (TX) y PA3 (RX)
  Serial2.begin(115200);

  // Inicializar el puerto serie de Hardware (UART1) para la comunicación RS-485
  // PA9 (TX) y PA10 (RX)
  Serial1.begin(9600);

  // Configurar el pin de control del MAX485 como salida
  pinMode(MAX485_DE_RE, OUTPUT);

  // Poner y mantener el MAX485 en modo de recepción
  digitalWrite(MAX485_DE_RE, LOW);

  Serial2.println("Esclavo RS-485 inicializado. Esperando mensajes...");
}

void loop() {
  // Comprobar si hay datos disponibles en el bus RS-485
  if (Serial1.available() > 0) {
    // Leer el mensaje completo hasta el carácter de nueva línea
    String receivedMessage = Serial1.readStringUntil('\n');

    // Mostrar el mensaje recibido en el monitor serie del esclavo
    Serial2.print("Mensaje recibido: ");
    Serial2.println(receivedMessage);
  }
}
