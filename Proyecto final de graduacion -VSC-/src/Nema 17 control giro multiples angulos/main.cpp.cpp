#include "AS5600.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

// --- Constantes ---
const float RAW_A_GRADOS = 360.0 / 4096.0;
const int pinPaso = PA5;
const int pinDireccion = PA6;
const int pinHabilitar =
    PA7; // Conectar al pin EN del A4988. LOW lo habilita, HIGH lo deshabilita.

// --- PARÁMETROS A AJUSTAR ---
// Si el motor no gira el ángulo exacto, revisa estos 3 valores:
const float PASOS_POR_REV_MOTOR =
    200.0;                 // Comunes: 200 (1.8°/paso) o 400 (0.9°/paso)
const int MICROPASOS = 32; // Ajustar según la configuración de micropasos del
                           // driver (DRV8825: MODE0, MODE1, MODE2)
const float RELACION_REDUCTOR =
    39.0; // Relación de tu caja reductora (ej: 37.0 para 37:1)

// *** NUEVO: Parámetro para compensar el backlash (juego mecánico) ***
// Ajusta este valor en grados. Si al cambiar de dirección el motor se pasa o no
// llega, modifica este valor. Un valor positivo es lo normal.
const float BACKLASH_COMPENSATION_GRADOS =
    1.5; // Grados a añadir para compensar
// --- FIN DE PARÁMETROS A AJUSTAR ---

const float PASOS_POR_REV_MOTOR_EFECTIVO = PASOS_POR_REV_MOTOR * MICROPASOS;
const float PASOS_POR_REV_SALIDA =
    PASOS_POR_REV_MOTOR_EFECTIVO *
    RELACION_REDUCTOR; // Pasos necesarios para una revolución completa en el
                       // eje de salida
const float PASOS_MOTOR_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;
const int MAX_PUNTOS_LOG = 500; // Número máximo de puntos a registrar. ¡Ajustar
                                // según la RAM de la Blue Pill!

// Velocidad y aceleración reducidas para evitar pérdida de pasos. Ajústalas
// según tu motor.
const float VELOCIDAD_MAXIMA = 800 * MICROPASOS;
const float ACELERACION = 300 * MICROPASOS;
const float TOLERANCIA_ANGULO = 1;

// --- Objetos
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder;

// --- Variables Globales ---
long vueltasCompletas = 0;
long lecturaEncoderAnterior = 0;
long encoderOffset =
    0; // Almacena el valor crudo del encoder en la posición Home.
bool homingCompletado = false;
int homingDirection = 0; // 0=stop, 1=derecha, -1=izquierda
bool movimientoEnProgreso = false;
float anguloObjetivoRelativoActual = 0.0;
int lastMoveDirection =
    0; // Para compensación de backlash. 1=positivo, -1=negativo.
float log_angulos[MAX_PUNTOS_LOG];
unsigned long log_tiempo[MAX_PUNTOS_LOG];
int puntos_log_actuales = 0;
bool registrando_datos = false;

// ==========================================================
// --- DEFINICIÓN DE FUNCIONES AUXILIARES (PUESTAS ARRIBA) ---
// ==========================================================

// Declaración adelantada para resolver dependencia de funciones
long leerPosicionContinuaEncoder();

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
  float diferencia =
      normalizarAngulo(anguloFinal) - normalizarAngulo(anguloInicial);
  if (diferencia > 180.0) {
    diferencia -= 360.0;
  } else if (diferencia < -180.0) {
    diferencia += 360.0;
  }
  return diferencia;
}

// Lee el valor crudo del encoder (0-4095)
long leerPasosCrudosEncoder() { return encoder.readAngle(); }

// Lee el ángulo del encoder y lo convierte a grados.

float leerAnguloEncoderEnGrados() {
  // Ahora devuelve el ángulo relativo al offset del Home
  long posicionRelativa = leerPosicionContinuaEncoder() - encoderOffset;
  return (float)posicionRelativa * RAW_A_GRADOS;
}

// Lee y actualiza la posición continua del encoder, manejando el
// desbordamiento.
long leerPosicionContinuaEncoder() {
  long lecturaCrudaActual = leerPasosCrudosEncoder();
  long diferenciaCruda = lecturaCrudaActual - lecturaEncoderAnterior;
  if (diferenciaCruda < -2048) {
    vueltasCompletas++;
  } else if (diferenciaCruda > 2048) {
    vueltasCompletas--;
  }
  lecturaEncoderAnterior = lecturaCrudaActual;
  return (vueltasCompletas * 4096) + lecturaCrudaActual;
}

// Mueve el motor a una 'posicionObjetivoLogico' (en micropasos) y espera a que
// termine.
void correrStepperHastaPosicion(long posicionObjetivoLogico) {
  motorPasoAPaso.moveTo(posicionObjetivoLogico);
  Serial2.print("Moviendo stepper a posicion logica: ");
  Serial2.println(posicionObjetivoLogico);

  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();
  }
  Serial2.print("Movimiento stepper completado. Posicion logica final: ");
  Serial2.println(motorPasoAPaso.currentPosition());
  delay(200);
}

// *** NUEVA FUNCIÓN: Mueve el motor aplicando compensación de backlash ***
void moverConCompensacion(long posicionLogicaObjetivo) {
  // Calcular la compensación en pasos del motor
  long backlash_steps =
      round(BACKLASH_COMPENSATION_GRADOS * PASOS_MOTOR_POR_GRADO_SALIDA);
  // Determinar la dirección del movimiento actual
  int currentDirection = 0;

  if (posicionLogicaObjetivo > motorPasoAPaso.currentPosition()) {
    currentDirection = 1; // Movimiento positivo
  } else if (posicionLogicaObjetivo < motorPasoAPaso.currentPosition()) {

    currentDirection = -1; // Movimiento negativo
  }

  // Aplicar compensación SOLO si la dirección cambia y no es el primer
  // movimiento
  if (currentDirection != 0 && lastMoveDirection != 0 &&
      currentDirection != lastMoveDirection) {
    Serial2.print("Compensando backlash. Direccion: ");
    Serial2.println(currentDirection == 1 ? "NEG -> POS" : "POS -> NEG");
    // Mover un poco más allá en la dirección del movimiento para eliminar la
    // holgura
    long posicionCompensada =
        posicionLogicaObjetivo + (currentDirection * backlash_steps);
    Serial2.print("   Moviendo a posicion sobrepasada: ");
    Serial2.println(posicionCompensada);
    correrStepperHastaPosicion(posicionCompensada); // Movimiento de sobrepaso
    delay(100); // Pequeña pausa para asentar la mecánica
  }

  // Mover a la posición final deseada (esto también corrige el sobrepaso si se
  // aplicó)
  correrStepperHastaPosicion(posicionLogicaObjetivo);
  // Actualizar la última dirección de movimiento solo si hubo un movimiento
  // real
  if (currentDirection != 0) {
    lastMoveDirection = currentDirection;
  }
}

// Imprime el menú de opciones para el usuario.
void imprimirIndicacion() {
  Serial2.println("------------------------------------");
  Serial2.println("d: +90 | i: -90 | e: +45 | q: -45");
}

// Inicia el proceso de registro de datos para un nuevo ensayo.
void iniciarRegistro() {
  puntos_log_actuales = 0;
  registrando_datos = true;
  Serial2.println("OK;Registro iniciado.");
}

// Detiene y envía los datos registrados a MATLAB.
void finalizarYEnviarRegistro() {
  registrando_datos = false;
  Serial2.println("--- INICIO_DATOS ---");
  for (int i = 0; i < puntos_log_actuales; i++) {
    Serial2.print(log_tiempo[i]);
    Serial2.print(",");
    Serial2.println(log_angulos[i], 4); // Enviar con 4 decimales para precisión
  }
  Serial2.println("--- FIN_DATOS ---");
  puntos_log_actuales = 0; // Limpiar para el próximo registro
}

// La función principal de movimiento que ahora también registra datos.
void moverYRegistrar(float anguloObjetivo) {
  long pasosRelativosMotor =
      round(anguloObjetivo * PASOS_MOTOR_POR_GRADO_SALIDA);
  long posicionObjetivo =
      motorPasoAPaso.currentPosition() + pasosRelativosMotor;
  motorPasoAPaso.moveTo(posicionObjetivo);
  unsigned long tiempo_inicio_mov = millis();

  while (motorPasoAPaso.distanceToGo() != 0) {
    motorPasoAPaso.run();
    // Registrar datos a intervalos regulares (ej. cada 10ms)
    if (registrando_datos && (millis() % 10 == 0) &&
        (puntos_log_actuales < MAX_PUNTOS_LOG)) {
      // Leemos la posición REAL del encoder en el eje de salida
      long pos_continua_actual = leerPosicionContinuaEncoder() - encoderOffset;
      float angulo_real_salida = (float)pos_continua_actual * RAW_A_GRADOS;
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
  Serial2.begin(115200);
  while (!Serial2) {
    ; // Esperar a que el puerto serie se conecte
  }
  Serial2.println(
      "\n--- Inicio de la secuencia de Setup (v3 - Home Virtual Cero) ---");

  // 1. Inicializar periféricos principales primero.
  Serial2.println("Inicializando bus I2C (Wire.begin())...");
  Wire.begin();
  delay(100); // Pausa para que el bus I2C se estabilice.

  // 2. Configurar pines de GPIO para el motor.
  Serial2.println("Configurando pines del motor...");
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  pinMode(pinHabilitar, OUTPUT);
  digitalWrite(pinHabilitar, HIGH); // Deshabilitar motor.

  // 3. Configurar el encoder.
  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);
  Serial2.println("Librería de encoder configurada.");

  // 4. Bucle de reintentos para la conexión (sin el Wire.end() previo).
  int maxRetries = 10;
  bool encoderConnected = false;
  Serial2.println("Iniciando intentos de conexión con el encoder...");

  for (int i = 0; i < maxRetries; i++) {
    // La llamada a isConnected() es la que realmente prueba la comunicación
    // I2C.
    if (encoder.isConnected()) {
      encoderConnected = true;
      Serial2.print("Intento [");
      Serial2.print(i + 1);
      Serial2.print("]: ");
      Serial2.println("¡Conexión exitosa!");
      break;
    } else {
      Serial2.print("Intento [");
      Serial2.print(i + 1);
      Serial2.print("]: ");
      Serial2.println("Fallo.");
      delay(500); // Esperar antes de reintentar.
    }
  }

  // 5. Verificación final.
  if (!encoderConnected) {
    Serial2.println("*****************************************************");
    Serial2.println(
        "Error Crítico: No se pudo conectar con el encoder AS5600.");
    Serial2.println("El sistema se detendrá.");
    Serial2.println("*****************************************************");
    while (1) {
    }
  }

  Serial2.println("Encoder detectado y listo.");
  Serial2.println("--- MODO CALIBRACION ---");
  Serial2.println("Use 'j' (izquierda) y 'k' (derecha) para mover el motor.");
  Serial2.println(
      "Presione 'h' para detener el movimiento y establecer el Home.");
  Serial2.println("------------------------------------");
}

// ==========================================================
// --- LOOP PRINCIPAL ---
// ==========================================================

void loop() {
  // --- FASE 1: ESPERANDO CALIBRACIÓN DE HOME ---
  if (!homingCompletado) {
    // Si el motor debe moverse para el homing, se ejecuta run()
    if (homingDirection != 0) {
      motorPasoAPaso.run(); // Mueve el motor hacia el objetivo lejano
    }

    // --- Manejo de comandos seriales para el homing ---
    if (Serial2.available() > 0) {
      char comando = tolower(Serial2.read());

      switch (comando) {
      case 'j': // Mover a la izquierda
        if (homingDirection != -1) {
          Serial2.println(
              "Homing: Moviendo a la izquierda. Presione 'h' para fijar home.");
          digitalWrite(pinHabilitar, LOW); // Habilitar motor
          motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA /
                                     4); // Velocidad de homing
          motorPasoAPaso.setAcceleration(ACELERACION / 4);
          motorPasoAPaso.moveTo(-2000000000); // Un número negativo muy grande
          homingDirection = -1;
        }
        break;

      case 'k': // Mover a la derecha
        if (homingDirection != 1) {
          Serial2.println(
              "Homing: Moviendo a la derecha. Presione 'h' para fijar home.");
          digitalWrite(pinHabilitar, LOW); // Habilitar motor
          motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA /
                                     4); // Velocidad de homing
          motorPasoAPaso.setAcceleration(ACELERACION / 4);
          motorPasoAPaso.moveTo(2000000000); // Un número positivo muy grande
          homingDirection = 1;
        }
        break;

      case 'h': // Detener y establecer Home
        Serial2.println("\nComando 'h' recibido. Deteniendo motor...");
        homingDirection = 0;
        motorPasoAPaso.stop(); // Detiene el motor usando la deceleración
        motorPasoAPaso
            .runToPosition(); // Espera a que el motor se detenga completamente
        Serial2.println(
            "Motor detenido. Estableciendo Home Virtual en CERO...");

        // --- Lógica para establecer el Home Virtual ---
        lecturaEncoderAnterior =
            leerPasosCrudosEncoder(); // Sincroniza la lectura continua
        vueltasCompletas = 0;
        encoderOffset = leerPosicionContinuaEncoder(); // Guarda el offset
                                                       // actual del encoder
        motorPasoAPaso.setCurrentPosition(
            0); // ¡CLAVE! Se establece la posición lógica a CERO.
        digitalWrite(pinHabilitar,
                     LOW); // Asegurarse que el motor sigue habilitado
        Serial2.print("Home establecido. Posicion actual: ");
        Serial2.print(leerAnguloEncoderEnGrados(), 2);
        Serial2.println(" GRADOS");
        Serial2.print("Offset del encoder (pasos crudos): ");
        Serial2.println(encoderOffset);
        Serial2.print("Posicion Logica del Stepper: ");
        Serial2.println(motorPasoAPaso.currentPosition());
        Serial2.println("--- MODO OPERACION NORMAL ---");

        // Restaurar velocidad y aceleración para la operación normal
        motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
        motorPasoAPaso.setAcceleration(ACELERACION);
        homingCompletado = true;
        imprimirIndicacion();
        break;
      }
    }
    return; // Salir del loop y volver a empezar si no se ha calibrado.
  }

  // --- FASE 2: OPERACIÓN NORMAL (después del homing) ---

  // Leer comandos solo si no hay un movimiento en curso
  if (Serial2.available() > 0) {
    char comando = tolower(Serial2.read());
    bool comandoValido = false;

    switch (comando) {
    case 'd':
      anguloObjetivoRelativoActual = 90.0;
      comandoValido = true;
      break;
    case 'i':
      anguloObjetivoRelativoActual = -90.0;
      comandoValido = true;
      break;
    case 'e':
      anguloObjetivoRelativoActual = 45.0;
      comandoValido = true;
      break;
    case 'q':
      anguloObjetivoRelativoActual = -45.0;
      comandoValido = true;
      break;
    }

    if (comandoValido) {
      Serial2.print("\nComando '");
      Serial2.print(comando);
      Serial2.print("': Moviendo ");
      Serial2.print(anguloObjetivoRelativoActual);
      Serial2.println(" grados en la salida.");

      // --- MOVIMIENTO DE IDA ---
      long pasosRelativosMotor =
          round(anguloObjetivoRelativoActual * PASOS_MOTOR_POR_GRADO_SALIDA);
      long posicionLogicaObjetivo =
          motorPasoAPaso.currentPosition() + pasosRelativosMotor;
      moverConCompensacion(posicionLogicaObjetivo);

      Serial2.println("   Llego al objetivo.");

      // --- VERIFICACIÓN CON ENCODER ---
      float anguloRealFinal = leerAnguloEncoderEnGrados();
      float errorGradosSalida = anguloObjetivoRelativoActual - anguloRealFinal;
      Serial2.print("   Angulo objetivo: ");
      Serial2.print(anguloObjetivoRelativoActual, 2);
      Serial2.print(" | Angulo real medido: ");
      Serial2.println(anguloRealFinal, 2);
      Serial2.print("   Error angular (grados salida): ");
      Serial2.println(errorGradosSalida, 2);
      if (abs(errorGradosSalida) > TOLERANCIA_ANGULO)
        Serial2.println("   ADVERTENCIA: Error angular grande!");

      // --- ESPERAR Y REGRESAR ---
      Serial2.println("   Esperando 1 segundo...");
      delay(1000);
      Serial2.println("   Regresando a posicion Home (0)...");
      moverConCompensacion(0); // Regresa a la posición lógica CERO

      // --- FIN DEL CICLO: Mostrar estado y esperar nuevo comando ---
      float anguloFinalTrasRegreso = leerAnguloEncoderEnGrados();
      Serial2.print("   Posicion final tras regreso: ");
      Serial2.print(anguloFinalTrasRegreso, 2);
      Serial2.println(" GRADOS");
      Serial2.print("   Posicion logica final: ");
      Serial2.println(motorPasoAPaso.currentPosition());
      imprimirIndicacion();
    }
  }
}
