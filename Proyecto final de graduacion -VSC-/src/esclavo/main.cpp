#include <Arduino.h>

// ========================================================================
// IMPORTANTE: CAMBIAR ESTE VALOR ANTES DE SUBIR EL CÓDIGO A CADA ESCLAVO
// Usa 2 para el Esclavo 2, y 3 para el Esclavo 3.
#define SLAVE_ID 3
// ========================================================================

// --- Pines de Hardware ---
const int LED_PIN = PC13;      // LED incorporado en la Blue Pill.
const int RS485_DE_PIN = PA8; // Pin para controlar el transceiver RS-485 (Driver Enable).

void setup() {
  // Configurar pines
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Inicia apagado (lógica invertida)
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW); // Inicia en modo recepción

  // Iniciar comunicación serial para el bus RS-485
  Serial1.begin(9600);
}

void loop() {
  // Verificar si hay datos disponibles en el bus RS-485 (estando en modo recepción)
  if (Serial1.available() > 0) {
    String message = Serial1.readStringUntil('\n');
    message.trim();

    // --- Procesamiento del Mensaje ---
    int colonIndex = message.indexOf(':');

    // Si el formato es inválido, se ignora. El maestro lo detectará como timeout.
    if (colonIndex == -1) {
      return;
    }

    int receivedId = message.substring(0, colonIndex).toInt();

    // Verificar si el mensaje es para este esclavo
    if (receivedId == SLAVE_ID) {
      String cmdStr = message.substring(colonIndex + 1);
      String response;

      // Procesar el comando y preparar la respuesta
      if (cmdStr.equalsIgnoreCase("TOGGLE_LED")) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        response = String(SLAVE_ID) + ":OK";
      } else {
        response = String(SLAVE_ID) + ":ERROR_UNKNOWN_COMMAND";
      }
      
      // --- Enviar la respuesta de vuelta al Maestro ---
      // 1. Poner el transceiver en modo transmisión
      digitalWrite(RS485_DE_PIN, HIGH);
      delay(5); // Pequeña espera para asegurar el cambio de estado del transceiver.

      // 2. Enviar la respuesta
      Serial1.println(response);
      
      // 3. Esperar a que se envíen todos los datos
      Serial1.flush(); 
      delay(5);

      // 4. Volver a poner el transceiver en modo recepción
      digitalWrite(RS485_DE_PIN, LOW);
    }
    // Si el mensaje no es para este esclavo, se ignora.
  }
}
