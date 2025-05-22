#include <Wire.h>

void setup() {
  Wire.begin();
  Serial1.begin(115200);
  while (!Serial1); // Espera a que el puerto serie esté listo
  Serial1.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial1.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // Wire.beginTransmission() devuelve 0 si se recibe ACK del dispositivo
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial1.print("I2C device found at address 0x");
      if (address < 16)
        Serial1.print("0");
      Serial1.print(address, HEX);
      Serial1.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial1.print("Unknown error at address 0x");
      if (address < 16)
        Serial1.print("0");
      Serial1.println(address, HEX);
    }
    // No reporta nada si error != 0 y != 4 (no se encontró dispositivo)
  }
  if (nDevices == 0)
    Serial1.println("No I2C devices found\n");
  else
    Serial1.println("done\n");

  delay(5000); // Espera 5 segundos para volver a escanear
}




