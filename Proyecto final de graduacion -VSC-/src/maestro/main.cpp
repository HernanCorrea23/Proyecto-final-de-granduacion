#include <Arduino.h>

// --- Pines de Hardware ---
const int LED_PIN = PC13;      // LED incorporado en la Blue Pill (usado por el Maestro como Esclavo 1)
const int RS485_DE_PIN = PA8; // Pin para controlar la dirección del transceiver (HIGH: Transmitir, LOW: Recibir)

// --- Prototipos de Funciones ---
void toggleMasterLed();
void sendCommandToSlave(int slaveId);

void setup() {
  // Configurar pines
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Inicia apagado (lógica invertida)
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW); // Inicia en modo recepción

  // Iniciar comunicación serial
  Serial1.begin(9600); // Bus RS-485
  Serial2.begin(9600); // Monitor con PC

  Serial2.println("==============================================");
  Serial2.println("|        NODO MAESTRO INICIALIZADO         |");
  Serial2.println("==============================================");
  Serial2.println("Presione '1', '2' o '3' para alternar el LED correspondiente.");
  Serial2.println("  1: Alterna el LED de este dispositivo (Maestro).");
  Serial2.println("  2: Envía comando para alternar LED al Esclavo 2.");
  Serial2.println("  3: Envía comando para alternar LED al Esclavo 3.");
  Serial2.println("----------------------------------------------");
}

void loop() {
  if (Serial2.available() > 0) {
    char command = Serial2.read();

    switch (command) {
      case '1':
        Serial2.println("\n> Comando '1' recibido. Alternando LED del Maestro.");
        toggleMasterLed();
        break;
      case '2':
        Serial2.println("\n> Comando '2' recibido. Enviando orden a Esclavo 2.");
        sendCommandToSlave(2);
        break;
      case '3':
        Serial2.println("\n> Comando '3' recibido. Enviando orden a Esclavo 3.");
        sendCommandToSlave(3);
        break;
      
      // Ignorar caracteres de nueva línea y otros caracteres no válidos
      case '\n':
      case '\r':
        break;
      default:
        if (isPrintable(command)) {
          Serial2.print("\n> Comando no válido: ");
          Serial2.println(command);
        }
        break;
    }
  }
}

/**
 * @brief Alterna el estado del LED del propio Maestro y lo reporta al monitor.
 */
void toggleMasterLed() {
  // La lógica del LED es invertida: LOW = ON, HIGH = OFF
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
  Serial2.print("  LED del Maestro (Esclavo 1) ahora está: ");
  Serial2.println(digitalRead(LED_PIN) == LOW ? "ENCENDIDO" : "APAGADO");
  Serial2.println("----------------------------------------------");
}

/**
 * @brief Envía el comando TOGGLE_LED a un esclavo y gestiona la respuesta.
 * @param slaveId El ID del esclavo de destino (2 o 3).
 */
void sendCommandToSlave(int slaveId) {
  String commandToSend = String(slaveId) + ":TOGGLE_LED";

  // 1. Poner el transceiver en modo transmisión
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(5);

  // 2. Enviar el mensaje por el bus RS-485
  Serial1.println(commandToSend);
  Serial1.flush();
  delay(5);

  // 3. Volver al modo recepción
  digitalWrite(RS485_DE_PIN, LOW);
  Serial2.print("  Comando enviado: [");
  Serial2.print(commandToSend);
  Serial2.println("]. Esperando respuesta...");

  // 4. Esperar y procesar la respuesta
  long startTime = millis();
  bool responseReceived = false;
  String response = "";

  while (millis() - startTime < 1000) { // Timeout de 1 segundo
    if (Serial1.available() > 0) {
      response = Serial1.readStringUntil('\n');
      response.trim();
      responseReceived = true;
      break;
    }
  }

  // 5. Interpretar el resultado
  if (responseReceived) {
    Serial2.print("  Respuesta recibida: [");
    Serial2.print(response);
    Serial2.println("]");

    int colonIndex = response.indexOf(':');
    if (colonIndex > 0) {
      int responseId = response.substring(0, colonIndex).toInt();
      String status = response.substring(colonIndex + 1);

      if (responseId == slaveId && status.equalsIgnoreCase("OK")) {
        Serial2.println("  -> ÉXITO: El esclavo ejecutó el comando correctamente.");
      } else {
        Serial2.println("  -> FALLO: La respuesta del esclavo no fue la esperada.");
      }
    } else {
      Serial2.println("  -> ERROR: El formato de la respuesta es inválido.");
    }
  } else {
    Serial2.print("  -> ERROR DE TIMEOUT: No se recibió respuesta del esclavo ");
    Serial2.print(slaveId);
    Serial2.println(".");
  }
  Serial2.println("----------------------------------------------");
}
