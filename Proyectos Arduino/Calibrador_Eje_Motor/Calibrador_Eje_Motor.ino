#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h"

// --- Constantes ---
const float RAW_A_GRADOS = 360.0 / 4096.0;
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7; 

const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 16;
const float RELACION_REDUCTOR = 37.0; 
const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA = PASOS_POR_REV_MOTOR_EFECTIVO * RELACION_REDUCTOR;
const float PASOS_MOTOR_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

const float VELOCIDAD_MAXIMA = 1500.0 * MICROPASOS;
const float ACELERACION = 800.0 * MICROPASOS;
const float TOLERANCIA_ANGULO = 2.5;

// --- Objetos ---
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder;

// --- Variables Globales ---
// Para manejar el Overflow del Encoder
long vueltasCompletas = 0;
long lecturaEncoderAnterior = 0;
long posicionContinuaEncoder = 0;
long posicionContinuaHome = 0;

// Variables de estado
bool homingCompletado = false;
bool movimientoEnProgreso = false;
float anguloObjetivoRelativoActual = 0.0;
long posicionLogicaStepperObjetivo = 0;


// ==========================================================
// --- DEFINICIÓN DE FUNCIONES AUXILIARES ---
// ==========================================================

// Lee el valor crudo del encoder (0-4095)
long leerPasosCrudosEncoder() {
  return encoder.readAngle(); // O encoder.rawAngle() si existe
}

// Lee y actualiza la posición continua del encoder, manejando el desbordamiento (overflow/wraparound).
void actualizarPosicionContinuaEncoder() {
  long lecturaCrudaActual = leerPasosCrudosEncoder();
  long diferenciaCruda = lecturaCrudaActual - lecturaEncoderAnterior;

  // Detectar salto hacia adelante (ej. de 4000 a 100)
  if (diferenciaCruda < -3000) { // Umbral grande para detectar el salto de 4095 a 0
    vueltasCompletas++;
  }
  // Detectar salto hacia atrás (ej. de 100 a 4000)
  else if (diferenciaCruda > 3000) {
    vueltasCompletas--;
  }

  lecturaEncoderAnterior = lecturaCrudaActual; // Actualizar para la próxima lectura
  posicionContinuaEncoder = (vueltasCompletas * 4096) + lecturaCrudaActual;
}

// Convierte una posición continua del encoder (cruda con vueltas) a la posición lógica del stepper.
long convertirPosContinuaAStprLogica(long posContinua) {
  return round((float)posContinua * PASOS_POR_REV_MOTOR_EFECTIVO / 4096.0);
}

// Imprime el menú de opciones para el usuario.
void imprimirIndicacion() {
  Serial1.println("------------------------------------");
  Serial1.println("d: +90 | i: -90 | e: +45 | q: -45");
}

// ==========================================================
// --- SETUP ---
// ==========================================================
void setup() {
  Serial1.begin(115200);
  Wire.begin();
  
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  
  digitalWrite(pinHabilitar, HIGH); // Mantener motor deshabilitado para alinear a mano.

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE); // Asegúrate que esta dirección sea la correcta.
  if (!encoder.isConnected()) {
    Serial1.println("ERROR: Encoder no detectado!");
    while (1);
  }
  
  Serial1.println("--- MODO CALIBRACION MANUAL ---");
  Serial1.println("1. Alinee manualmente las marcas.");
  Serial1.println("2. Envie la tecla 'h' para establecer el Home.");
  Serial1.println("------------------------------------");
}

// ==========================================================
// --- LOOP PRINCIPAL ---
// ==========================================================
void loop() {
  // --- FASE 1: ESPERANDO CALIBRACIÓN DE HOME ---
  if (!homingCompletado) {
    if (millis() % 500 < 20) { // Imprimir aprox. cada medio segundo para no saturar
        Serial1.print("\rAngulo actual para alinear (EJE MOTOR): ");
        Serial1.print((float)leerPasosCrudosEncoder() * RAW_A_GRADOS, 2);
        Serial1.print("   ");
    }

    if (Serial1.available() > 0) {
      char comando = Serial1.read();
      if (comando == 'h' || comando == 'H') {
        Serial1.println("\nComando 'h' recibido. Estableciendo Home...");
        
        // Inicializar el sistema de conteo de vueltas
        lecturaEncoderAnterior = leerPasosCrudosEncoder();
        vueltasCompletas = 0;
        posicionContinuaHome = lecturaEncoderAnterior; 
        
        // Sincronizar AccelStepper con la posición continua inicial
        long posInicialLogica = convertirPosContinuaAStprLogica(posicionContinuaHome);
        motorPasoAPaso.setCurrentPosition(posInicialLogica);
        
        digitalWrite(pinHabilitar, LOW); // Habilitar motor.
        
        Serial1.print("Home establecido en el angulo (GRADOS): "); Serial1.println((float)posicionContinuaHome * RAW_A_GRADOS, 2);
        Serial1.print("Posicion Continua Home (pasos crudos): "); Serial1.println(posicionContinuaHome);
        Serial1.print("Posicion Logica Stepper Inicial: "); Serial1.println(posInicialLogica);
        Serial1.println("--- MODO OPERACION NORMAL ---");
        
        motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
        motorPasoAPaso.setAcceleration(ACELERACION);
        
        homingCompletado = true;
        imprimirIndicacion();
      }
    }
    return; // Salir del loop y volver a empezar si no se ha calibrado.
  }

  // --- FASE 2: OPERACIÓN NORMAL (después del homing) ---

  // TAREA 1: ACTUALIZAR ESTADO CONSTANTEMENTE
  actualizarPosicionContinuaEncoder();
  motorPasoAPaso.run();

  // TAREA 2: VERIFICAR FIN DE MOVIMIENTO Y GESTIONAR SIGUIENTE PASO
  if (movimientoEnProgreso && motorPasoAPaso.distanceToGo() == 0) {
    // Si la posición actual del stepper coincide con el objetivo que le dimos...
    if (motorPasoAPaso.currentPosition() == posicionLogicaStepperObjetivo) {
        // ... significa que llegó al destino de +/- 45/90 grados.
        Serial1.println("   Llego al objetivo.");
        
        // VERIFICACIÓN CON ENCODER
        long pasosObjetivoRelativosMotor = round(anguloObjetivoRelativoActual * PASOS_MOTOR_POR_GRADO_SALIDA);
        float gradosObjetivoSalida = (float)pasosObjetivoRelativosMotor / PASOS_MOTOR_POR_GRADO_SALIDA;
        
        long movimientoContinuoRealizado = posicionContinuaEncoder - posicionContinuaHome;
        float gradosSalidaRealizados = (float)movimientoContinuoRealizado / (4096.0 / PASOS_POR_REV_MOTOR_EFECTIVO) / RELACION_REDUCTOR;
        
        float errorGradosSalida = gradosObjetivoSalida - gradosSalidaRealizados;

        Serial1.print("   Verificacion Pos Continua Encoder: "); Serial1.println(posicionContinuaEncoder);
        Serial1.print("   Movimiento de Salida Realizado (GRADOS): "); Serial1.println(gradosSalidaRealizados, 2);
        Serial1.print("   Error vs Esperado (grados salida): "); Serial1.println(errorGradosSalida, 2);
        if (abs(errorGradosSalida) > TOLERANCIA_ANGULO) Serial1.println("   ADVERTENCIA: Error angular grande!");
        
        // ESPERAR Y REGRESAR
        Serial1.println("   Esperando 1 segundo...");
        delay(1000);
        
        Serial1.println("   Regresando a posicion Home...");
        long posicionLogicaHome = convertirPosContinuaAStprLogica(posicionContinuaHome);
        posicionLogicaStepperObjetivo = posicionLogicaHome; // El nuevo objetivo es volver al home
        motorPasoAPaso.moveTo(posicionLogicaStepperObjetivo);

    } else {
        // ... significa que llegó al destino de regreso (el home).
        Serial1.println("   Llego de vuelta al Home.");
        
        // Re-calibrar el home para el próximo ciclo
        posicionContinuaHome = posicionContinuaEncoder;
        long posicionLogicaHomeActualizada = convertirPosContinuaAStprLogica(posicionContinuaHome);
        motorPasoAPaso.setCurrentPosition(posicionLogicaHomeActualizada); // Resincronizar
        
        Serial1.print("   Posicion resincronizada. Nuevo Home Continuo: "); Serial1.println(posicionContinuaHome);
        
        movimientoEnProgreso = false; // Fin del ciclo completo
        imprimirIndicacion();
    }
  }

  // TAREA 3: LEER NUEVOS COMANDOS (SI NO HAY MOVIMIENTO EN PROGRESO)
  if (!movimientoEnProgreso && Serial1.available() > 0) {
    char comando = Serial1.read();
    comando = tolower(comando);
    
    bool comandoValido = false;
    switch(comando) {
      case 'd': anguloObjetivoRelativoActual = 90.0; comandoValido = true; break;
      case 'i': anguloObjetivoRelativoActual = -90.0; comandoValido = true; break;
      case 'e': anguloObjetivoRelativoActual = 45.0; comandoValido = true; break;
      case 'q': anguloObjetivoRelativoActual = -45.0; comandoValido = true; break;
    }

    if (comandoValido) {
      movimientoEnProgreso = true;
      Serial1.print("\nComando '"); Serial1.print(comando); Serial1.print("': Moviendo "); 
      Serial1.print(anguloObjetivoRelativoActual); Serial1.println(" grados en la salida.");

      long pasosRelativosMotor = round(anguloObjetivoRelativoActual * PASOS_MOTOR_POR_GRADO_SALIDA);
      long posicionContinuaObjetivo = posicionContinuaHome + round((float)pasosRelativosMotor * 4096.0 / PASOS_POR_REV_MOTOR_EFECTIVO);
      
      posicionLogicaStepperObjetivo = convertirPosContinuaAStprLogica(posicionContinuaObjetivo);
      
      motorPasoAPaso.moveTo(posicionLogicaStepperObjetivo);
    }
  }
}