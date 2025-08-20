#include <Arduino.h>

// Definir el pin de control para el módulo MAX485 (DE/RE)
// Conectar DE y RE juntos a este pin.
const int MAX485_DE_RE = PA8;

// Variable para el contador de mensajes
int messageCounter = 0;

void setup() {
  // Inicializar el puerto serie (UART2) para monitoreo
  // PA2 (TX) y PA3 (RX)
  Serial2.begin(115200);
  
  // Inicializar el puerto serie de Hardware (UART1) para la comunicación RS-485
  // PA9 (TX) y PA10 (RX)
  Serial1.begin(9600);

  // Configurar el pin de control del MAX485 como salida
  pinMode(MAX485_DE_RE, OUTPUT);

  // Poner el MAX485 en modo de recepción por defecto
  digitalWrite(MAX485_DE_RE, LOW);

  Serial2.println("Maestro RS-485 inicializado.");
}

void loop() {
  // Construir el mensaje a enviar
  String message = "Mensaje numero: " + String(messageCounter);

  // 1. Poner el MAX485 en modo de transmisión
  digitalWrite(MAX485_DE_RE, HIGH);
  
  // Pequeña demora para asegurar que el modo de transmisión está activo
  delay(10);

  // 2. Enviar el mensaje por el bus RS-485
  Serial1.println(message);
  
  // 3. Esperar a que se complete la transmisión de todos los bytes
  Serial1.flush();

  // 4. Volver a poner el MAX485 en modo de recepción (baja impedancia)
  digitalWrite(MAX485_DE_RE, LOW);

  // Mostrar en el monitor serie del maestro el mensaje enviado
  Serial2.print("Mensaje enviado: ");
  Serial2.println(message);

  // Incrementar el contador
  messageCounter++;

  // Esperar 2 segundos antes de enviar el siguiente mensaje
  delay(2000);
}