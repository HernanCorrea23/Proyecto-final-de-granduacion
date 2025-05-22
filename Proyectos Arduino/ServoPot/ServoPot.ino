#include //Incluimos la libreria del motor
#include <Servo.h>

Servo servoMotor; // Creamos el objeto motor para poder controlarlo

int potenciometroPin = 0; // Establecemos el pin analogico para controlar el potenciometr
int val; // variable para leer los valores del potenciometro

void setup() {
servoMotor.attach(9); // establecemos el pin del motor en el pin digital 9
Serial.begin(9600);

}

void loop() {
val = analogRead(potenciometroPin); // leemos el valor del potenciometro (valor entre 0 y 1023)
val = map(val, 0, 1023, 0, 180); // transformamos el valor del potenciometro para usarlo en el motor (valor entre 0 y 180)
Serial.println(val);
servoMotor.write(val); // una vez transformado el valor, se lo mandamos al servomotor para moverlo
delay(15); // Esperamos la respuesta del servomotor
}