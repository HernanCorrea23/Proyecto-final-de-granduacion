// --- CÓDIGO DEL ESCÁNER I²C ---
#include <Arduino.h>
#include <Wire.h>

void setup() {
    // Usamos Serial2 para el monitor, como en tus pruebas anteriores
    Serial1.begin(115200);
    while (!Serial1) {}

    // Inicia el bus I2C en los pines por defecto (PB6=SCL, PB7=SDA)
    Wire.begin(); 

    Serial1.println("\nIniciando escaneo de dispositivos I2C...");
}

void loop() {
    byte error, address;
    int nDevices;

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial1.print("Dispositivo I2C encontrado en la direccion 0x");
            if (address < 16) {
                Serial1.print("0");
            }
            Serial1.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            Serial1.print("Error desconocido en la direccion 0x");
            if (address < 16) {
                Serial1.print("0");
            }
            Serial1.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial1.println("No se encontraron dispositivos I2C. Revisar cableado.");
    } else {
        Serial1.println("Escaneo finalizado.");
    }

    delay(5000); // Repetir el escaneo cada 5 segundos
}