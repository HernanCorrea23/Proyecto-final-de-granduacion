#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h"

// --- Constantes ---
const float RAW_A_GRADOS = 360.0 / 4096.0;
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar = PA7; // Conectar al pin EN del A4988. LOW lo habilita, HIGH lo deshabilita.

const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 16;
const float RELACION_REDUCTOR = 37.0; 
const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA = PASOS_POR_REV_MOTOR_EFECTIVO * RELACION_REDUCTOR;
const float PASOS_MOTOR_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;
const int MAX_PUNTOS_LOG = 500; // Número máximo de puntos a registrar. ¡Ajustar según la RAM de la Blue Pill!

const float VELOCIDAD_MAXIMA = 1000.0 * MICROPASOS;
const float ACELERACION = 600.0 * MICROPASOS;
const float TOLERANCIA_ANGULO = 2.5;

// --- Objetos ---
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder;

// --- Variables Globales ---
long vueltasCompletas = 0;
long lecturaEncoderAnterior = 0;
long posicionContinuaHome = 0;

bool homingCompletado = false;
bool movimientoEnProgreso = false;
float anguloObjetivoRelativoActual = 0.0;

float log_angulos[MAX_PUNTOS_LOG];
unsigned long log_tiempo[MAX_PUNTOS_LOG];
int puntos_log_actuales = 0;
bool registrando_datos = false;


// ==========================================================
// --- DEFINICIÓN DE FUNCIONES AUXILIARES (PUESTAS ARRIBA) ---
// ==========================================================

// Normaliza un ángulo para que esté en el rango [0, 360) grados.
float normalizarAngulo(float angulo) {
  angulo = fmod(angulo, 360.0);
  if (angulo < 0) {
    angulo += 360.0;
  }
  return angulo;
}

// Calcula la diferencia angular más corta (con signo) entre dos ángulos.
float calcularDiferenciaAngular(float anguloFinal, float anguloInicial) {
  float diferencia = normalizarAngulo(anguloFinal) - normalizarAngulo(anguloInicial);
  if (diferencia > 180.0) {
    diferencia -= 360.0;
  } else if (diferencia < -180.0) {
    diferencia += 360.0;
  }
  return diferencia;
}

// Lee el valor crudo del encoder (0-4095)
long leerPasosCrudosEncoder() {
  return encoder.readAngle();
}

// Lee el ángulo del encoder y lo convierte a grados.
float leerAnguloEncoderEnGrados() {
  return (float)leerPasosCrudosEncoder() * RAW_A_GRADOS;
}


// Lee y actualiza la posición continua del encoder, manejando el desbordamiento.
long leerPosicionContinuaEncoder() {
  long lecturaCrudaActual = leerPasosCrudosEncoder();
  long diferenciaCruda = lecturaCrudaActual - lecturaEncoderAnterior;

  if (diferenciaCruda < -2048) {
    vueltasCompletas++;
  }
  else if (diferenciaCruda > 2048) {
    vueltasCompletas--;
  }

  lecturaEncoderAnterior = lecturaCrudaActual;
  return (vueltasCompletas * 4096) + lecturaCrudaActual;
}

// Mueve el motor a una 'posicionObjetivoLogico' (en micropasos) y espera a que termine.
void correrStepperHastaPosicion(long posicionObjetivoLogico) {
  motorPasoAPaso.moveTo(posicionObjetivoLogico);
  Serial1.print("Moviendo stepper a posicion logica: "); Serial1.println(posicionObjetivoLogico);
  
  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();
  }
  
  Serial1.print("Movimiento stepper completado. Posicion logica final: "); Serial1.println(motorPasoAPaso.currentPosition());
  delay(200);
}

// Imprime el menú de opciones para el usuario.
void imprimirIndicacion() {
  Serial1.println("------------------------------------");
  Serial1.println("d: +90 | i: -90 | e: +45 | q: -45");
}

// Inicia el proceso de registro de datos para un nuevo ensayo.
void iniciarRegistro() {
  puntos_log_actuales = 0;
  registrando_datos = true;
  Serial1.println("OK;Registro iniciado.");
}

// Detiene y envía los datos registrados a MATLAB.
void finalizarYEnviarRegistro() {
  registrando_datos = false;
  Serial1.println("--- INICIO_DATOS ---");
  for (int i = 0; i < puntos_log_actuales; i++) {
    Serial1.print(log_tiempo[i]);
    Serial1.print(",");
    Serial1.println(log_angulos[i], 4); // Enviar con 4 decimales para precisión
  }
  Serial1.println("--- FIN_DATOS ---");
  puntos_log_actuales = 0; // Limpiar para el próximo registro
}

// La función principal de movimiento que ahora también registra datos.
void moverYRegistrar(float anguloObjetivo) {
  long pasos_relativos_motor = round(anguloObjetivo * PASOS_MOTOR_POR_GRADO_SALIDA);
  motorPasoAPaso.move(pasos_relativos_motor);

  unsigned long tiempo_inicio_mov = millis();

  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();

    // Registrar datos a intervalos regulares (ej. cada 10ms)
    if (registrando_datos && (millis() % 10 == 0) && (puntos_log_actuales < MAX_PUNTOS_LOG)) {
      // Leemos la posición REAL del encoder en el eje de salida
      long pos_continua_actual = leerPosicionContinuaEncoder() - posicionContinuaHome;
      float angulo_real_salida = (float)pos_continua_actual / (PASOS_POR_REV_SALIDA / 360.0);

      log_angulos[puntos_log_actuales] = angulo_real_salida;
      log_tiempo[puntos_log_actuales] = millis() - tiempo_inicio_mov;
      puntos_log_actuales++;
    }
  }
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
  
  digitalWrite(pinHabilitar, HIGH); // Deshabilitar motor para alinear a mano.

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);
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
    if (millis() % 500 < 20) {
        Serial1.print("\rAngulo actual para alinear (EJE MOTOR): ");
        Serial1.print(leerAnguloEncoderEnGrados(), 2);
        Serial1.print("   ");
    }

    if (Serial1.available() > 0) {
      char comando = Serial1.read();
      if (comando == 'h' || comando == 'H') {
        Serial1.println("\nComando 'h' recibido. Estableciendo Home...");
        
        lecturaEncoderAnterior = leerPasosCrudosEncoder();
        vueltasCompletas = 0;
        posicionContinuaHome = lecturaEncoderAnterior; 
        
        long posInicialLogica = round((float)posicionContinuaHome * PASOS_POR_REV_MOTOR_EFECTIVO / 4096.0);
        motorPasoAPaso.setCurrentPosition(posInicialLogica);
        
        digitalWrite(pinHabilitar, LOW); // Habilitar motor.
        
        Serial1.print("Home establecido en el angulo (GRADOS): "); Serial1.println(leerAnguloEncoderEnGrados(), 2);
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
  
  // Ejecutar el stepper si hay un movimiento en curso.
  if (movimientoEnProgreso) {
    motorPasoAPaso.run();
  }

  // Verificar si el movimiento de ida se ha completado
  if (movimientoEnProgreso && motorPasoAPaso.distanceToGo() == 0) {
      Serial1.println("   Llego al objetivo.");
      movimientoEnProgreso = false;

      // VERIFICACIÓN CON ENCODER
      long posicionContinuaFinal = leerPosicionContinuaEncoder();
      long pasosComandadosNetos = round(anguloObjetivoRelativoActual * PASOS_MOTOR_POR_GRADO_SALIDA);
      long posicionContinuaObjetivoEsperada = posicionContinuaHome + round((float)pasosComandadosNetos * 4096.0 / PASOS_POR_REV_MOTOR_EFECTIVO);
      long errorPasosContinuos = posicionContinuaObjetivoEsperada - posicionContinuaFinal;
      float errorGradosSalida = (float)errorPasosContinuos / (4096.0 / PASOS_POR_REV_MOTOR_EFECTIVO) / PASOS_MOTOR_POR_GRADO_SALIDA;
      
      Serial1.print("   Verificacion Pos Continua Encoder: "); Serial1.println(posicionContinuaFinal);
      Serial1.print("   Error vs Esperado (grados salida): "); Serial1.println(errorGradosSalida, 2);
      if (abs(errorGradosSalida) > TOLERANCIA_ANGULO) Serial1.println("   ADVERTENCIA: Error angular grande!");
      
      // ESPERAR Y REGRESAR
      Serial1.println("   Esperando 1 segundo...");
      delay(1000);
      Serial1.println("   Regresando a posicion Home...");
      
      long posicionLogicaHome = round((float)posicionContinuaHome * PASOS_POR_REV_MOTOR_EFECTIVO / 4096.0);
      correrStepperHastaPosicion(posicionLogicaHome);
      
      // FIN DEL CICLO, resincronizar y esperar nuevo comando
      posicionContinuaHome = leerPosicionContinuaEncoder(); 
      posicionLogicaHome = round((float)posicionContinuaHome * PASOS_POR_REV_MOTOR_EFECTIVO / 4096.0);
      motorPasoAPaso.setCurrentPosition(posicionLogicaHome);
      
      Serial1.print("   Posicion resincronizada. Nuevo Home Continuo: "); Serial1.println(posicionContinuaHome);
      imprimirIndicacion();
  }

  // Leer comandos solo si no hay un movimiento en curso
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
      long posicionLogicaObjetivo = round((float)posicionContinuaObjetivo * PASOS_POR_REV_MOTOR_EFECTIVO / 4096.0);

      correrStepperHastaPosicion(posicionLogicaObjetivo);
    }
  }
}