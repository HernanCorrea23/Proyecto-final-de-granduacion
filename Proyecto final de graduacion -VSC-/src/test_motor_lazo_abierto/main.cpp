#include <Arduino.h>
#include <AccelStepper.h>

// --- Definiciones de Pines ---
const int STEP_PIN = PA5;
const int DIR_PIN = PA6;
const int EN_PIN = PA7;

// --- Parámetros del Motor ---
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const long STEPS_PER_REVOLUTION = MOTOR_STEPS_PER_REV * MICROSTEPS;

// --- Creación del objeto AccelStepper ---
// Interfaz: 1 (Driver), Pin de Paso, Pin de Dirección
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Prototipos de Funciones ---
void runTestSequence();

// --- Variable de Estado ---
bool sequenceHasRun = false;

void setup() {
  // Inicializar comunicación serial para monitorización
  Serial1.begin(115200);
  while (!Serial1) {
    ; // Esperar a que el puerto serie se conecte
  }
  Serial1.println("--- Inicio de Prueba de Motor en Lazo Abierto ---");

  // Configurar pin de habilitación. La librería se encargará de controlarlo.
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // Empezar con el motor deshabilitado (EN es activo bajo)

  // Configurar parámetros de AccelStepper
  stepper.setEnablePin(EN_PIN);
  // Invertir lógica del pin EN, ya que es activo bajo (LOW para habilitar)
  stepper.setPinsInverted(false, false, true); 
  
  // Establecer velocidad y aceleración máximas en (micro)pasos/segundo
  stepper.setMaxSpeed(6400); // 6400 micropasos/seg = 2 rev/seg
  stepper.setAcceleration(3200); // 3200 micropasos/seg^2

  Serial1.println("Motor configurado. La secuencia se ejecutará una vez.");
}

void loop() {
  if (!sequenceHasRun) {
    runTestSequence();
    sequenceHasRun = true;
  }
}

void runTestSequence() {
  // Habilitar el motor antes de mover
  stepper.enableOutputs();
  
  long targetSteps = 8 * STEPS_PER_REVOLUTION;

  // 1. Girar 5 revoluciones en un sentido
  Serial1.print("Paso 1: Girando 8 revoluciones (");
  Serial1.print(targetSteps);
  Serial1.println(" pasos)...");
  
  stepper.moveTo(targetSteps);
  stepper.runToPosition();
  
  Serial1.println("Movimiento completado.");

  // 2. Pausa de 1 segundo
  Serial1.println("Paso 2: Pausa de 1 segundo.");
  delay(1000);

  // 3. Girar 5 revoluciones en sentido contrario
  Serial1.println("Paso 3: Regresando a la posición inicial...");
  
  stepper.moveTo(0);
  stepper.runToPosition();
  
  Serial1.println("Posición inicial alcanzada.");

  // 4. Deshabilitar motor para ahorrar energía
  Serial1.println("--- Secuencia Finalizada ---");
  Serial1.println("Deshabilitando motor.");
  stepper.disableOutputs();

  // Bucle infinito para no repetir la secuencia
  while(1) {
    // Detener ejecución
  }
}
