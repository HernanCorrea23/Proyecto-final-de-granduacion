// #include <AccelStepper.h>
// #include <Wire.h>
// #include "AS5600.h" // La librería AS5600 que te funciona

// // --- Configuración de Pines ---
// const int pinPaso = PA5;
// const int pinDireccion = PA6;
// // Los pines I2C por defecto en BluePill suelen ser PB7 (SDA) y PB6 (SCL)

// // --- Configuración del Motor, Reductor y Encoder ---
// const float PASOS_POR_REV_MOTOR = 200.0;    // Pasos por revolución del NEMA 17 (típico 1.8 grados/paso)
// const int MICROPASOS = 16;                // Microstepping configurado en el A4988 (ej: 1, 2, 4, 8, 16)
// const float RELACION_REDUCTOR = 30.0;            // Relación de reducción de la base
// // Pasos del motor necesarios para una revolución completa del eje de SALIDA
// const float PASOS_POR_REV_SALIDA = PASOS_POR_REV_MOTOR * MICROPASOS * RELACION_REDUCTOR;
// // Pasos del motor necesarios para girar 90 grados el eje de SALIDA
// const long PASOS_PARA_90_GRADOS = round(PASOS_POR_REV_SALIDA / 4.0);
// // Pasos por grado en el eje de salida
// const float PASOS_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

// // --- Configuración de Movimiento (¡AJUSTAR ESTOS VALORES EXPERIMENTALMENTE!) ---
// const float VELOCIDAD_MAXIMA = 1500.0 * MICROPASOS; // Velocidad máxima en pasos/segundo (ajusta para tu motor/reductor)
// const float ACELERACION = 800.0 * MICROPASOS;    // Aceleración en pasos/segundo^2 (ajusta para suavidad)
// const float TOLERANCIA_ANGULO = 1.0;              // Tolerancia en grados para la verificación final

// // --- Creación de Objetos ---
// // Interfaz DRIVER: Se usa 1 pin para PASO y 1 pin para DIRECCION
// AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);

// // Objeto para el encoder AS5600
// AS5600 encoder; // O AS5600L si esa es la clase que te funciona

// // --- Variables Globales ---
// float anguloEncoderAbsInicial = 0.0; // Ángulo absoluto medido por el encoder al inicio (o al regresar)
// long posicionLogicaActual = 0;       // Posición lógica actual del stepper (en pasos)
// bool esperandoEntrada = true;        // Estado para saber si esperamos una 'A' o 'D' (o 'i' y 'd')

// // --- Funciones Auxiliares ---

// // Normaliza un ángulo para que esté en el rango [0, 360)
// float normalizarAngulo(float angulo) {
//   angulo = fmod(angulo, 360.0);
//   if (angulo < 0) {
//     angulo += 360.0;
//   }
//   return angulo;
// }

// // Calcula la diferencia más corta entre dos ángulos (resultado entre -180 y +180)
// float calcularDiferenciaAngular(float angulo1, float angulo2) {
//   float diferencia = normalizarAngulo(angulo1) - normalizarAngulo(angulo2);
//   if (diferencia > 180.0) {
//     diferencia -= 360.0;
//   } else if (diferencia <= -180.0) {
//     diferencia += 360.0;
//   }
//   return diferencia;
// }

// // Función para mover el stepper a la posición deseada (en pasos absolutos lógicos) y esperar
// void correrStepperHastaPosicion(long posicionObjetivo) {
//    motorPasoAPaso.moveTo(posicionObjetivo);
//    Serial1.print("Moviendo stepper a posicion logica: "); Serial1.println(posicionObjetivo);
//    // long distanciaInicial = motorPasoAPaso.distanceToGo(); // Para depurar
//    // long ultimaDistancia = distanciaInicial; // Para depurar
//    // unsigned long ultimoTiempoReporte = millis(); // Para depurar

//    while (motorPasoAPaso.distanceToGo() != 0) {
//      motorPasoAPaso.run(); // Ejecuta los pasos necesarios

//      // Opcional: Imprimir distancia restante cada cierto tiempo para ver progreso
//      // if (millis() - ultimoTiempoReporte > 500) {
//      //   long restante = motorPasoAPaso.distanceToGo();
//      //   Serial1.print("Pasos restantes: "); Serial1.println(restante);
//      //   ultimoTiempoReporte = millis();
//      // }
//    }
//    posicionLogicaActual = motorPasoAPaso.currentPosition(); // Actualizar posición lógica
//    Serial1.print("Movimiento stepper completado. Posicion logica final: "); Serial1.println(posicionLogicaActual);
//    delay(150); // Pequeña pausa para estabilizar mecánicamente
// }

// // Función para imprimir el mensaje de solicitud de entrada
// void imprimirIndicacion() {
//   Serial1.println("------------------------------------");
//   Serial1.println("Ingrese 'i' (izquierda 90 grados) o 'd' (derecha 90 grados):");
// }


// void setup() {
//   Serial1.begin(115200); // Usar una velocidad alta es recomendable
//   Wire.begin();        // Iniciar comunicación I2C

//   Serial1.println("Iniciando Setup...");

//   // Configurar pines del motor como salida
//   pinMode(pinPaso, OUTPUT);
//   pinMode(pinDireccion, OUTPUT);
//   digitalWrite(pinPaso, LOW);
//   digitalWrite(pinDireccion, LOW);

//   // Inicializar el encoder AS5600
//   // ¡¡¡ADAPTA ESTO A TU LIBRERÍA ESPECÍFICA SI ES NECESARIO!!!
//   // Si el código que te funcionó usaba encoder.begin(ALGUN_PIN), ponlo aquí.
//   encoder.begin();
//   // encoder.setDirection(AS5600_CLOCK_WISE); // O AS5600_COUNTER_CLOCK_WISE, ajusta según tu montaje
  
//   // Intenta verificar la conexión. Si tu librería no tiene isConnected(), prueba leyendo un ángulo.
//   bool encoderOK = encoder.isConnected(); 
//   /* // Alternativa de verificación
//   float anguloPrueba = encoder.readAngle();
//   if (anguloPrueba >= 0.0 && anguloPrueba <= 360.0) { // Una lectura válida
//       encoderOK = true;
//   } else {
//       encoderOK = false; // O si devuelve un código de error específico
//   }
//   */
//   if (!encoderOK) {
//       Serial1.println("ERROR: Encoder AS5600 NO detectado o falla al inicializar.");
//       Serial1.println("Verifique conexiones I2C (SDA->PB7, SCL->PB6), VCC (3.3V externo rec.), alimentacion y pull-ups.");
//       while(1); // Detener ejecución
//   }
//    Serial1.println("Encoder AS5600 detectado.");

//   // Leer la posición angular inicial ABSOLUTA del encoder
//   anguloEncoderAbsInicial = encoder.readAngle(); // O getAngle(), etc. Asegúrate que devuelva grados [0-360]
//   if (anguloEncoderAbsInicial < -0.1 ) { // Algunas librerías pueden devolver -1 en error, o valores fuera de rango
//       Serial1.println("ERROR al leer angulo inicial del encoder.");
//       while(1);
//   }
//   Serial1.print("Angulo inicial absoluto medido por Encoder: ");
//   Serial1.println(anguloEncoderAbsInicial);

//   // Configuración de AccelStepper
//   motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
//   motorPasoAPaso.setAcceleration(ACELERACION);
//   motorPasoAPaso.setCurrentPosition(0); // Definimos la posición LOGICA actual como 0 PASOS
//   posicionLogicaActual = 0;

//   Serial1.println("Setup completado.");
//   imprimirIndicacion(); // Pedir la primera entrada
// }

// void loop() {
//   // Solo procesar comandos si estamos esperando entrada
//   if (esperandoEntrada && Serial1.available() > 0) {
//     char comando = Serial1.read();
//     // command = toupper(command); // Convertir a mayúscula si prefieres 'D' e 'I'
//     comando = tolower(comando); // Usar minúsculas para 'd' e 'i'

//     long posicionLogicaPasosObjetivo = 0;
//     float anguloAbsolutoEsperado = 0.0;
//     bool comandoValido = false;

//     if (comando == 'd') { // Derecha
//       Serial1.println("\nComando 'd' recibido.");
//       posicionLogicaPasosObjetivo = PASOS_PARA_90_GRADOS;
//       anguloAbsolutoEsperado = normalizarAngulo(anguloEncoderAbsInicial + 90.0);
//       Serial1.print("Moviendo a +90 grados (Objetivo Pasos Logicos: "); Serial1.print(posicionLogicaPasosObjetivo);
//       Serial1.print(", Objetivo Angulo Abs Esperado: "); Serial1.print(anguloAbsolutoEsperado); Serial1.println(")");
//       comandoValido = true;

//     } else if (comando == 'i') { // Izquierda
//       Serial1.println("\nComando 'i' recibido.");
//       posicionLogicaPasosObjetivo = -PASOS_PARA_90_GRADOS;
//       anguloAbsolutoEsperado = normalizarAngulo(anguloEncoderAbsInicial - 90.0);
//       Serial1.print("Moviendo a -90 grados (Objetivo Pasos Logicos: "); Serial1.print(posicionLogicaPasosObjetivo);
//       Serial1.print(", Objetivo Angulo Abs Esperado: "); Serial1.print(anguloAbsolutoEsperado); Serial1.println(")");
//       comandoValido = true;
//     }

//     if (comandoValido) {
//       esperandoEntrada = false; // Dejar de esperar mientras se ejecuta

//       // --- Ejecutar Movimiento ---
//       // Calculamos la diferencia angular real y los pasos desde la posición actual del encoder
//       float anguloEncoderActualAntesDeMover = encoder.readAngle();
//       float diferenciaAngularReal = calcularDiferenciaAngular(anguloAbsolutoEsperado, anguloEncoderActualAntesDeMover);
//       long pasosRealesAMover = round(diferenciaAngularReal * PASOS_POR_GRADO_SALIDA);
//       long nuevaPosicionLogicaStepper = motorPasoAPaso.currentPosition() + pasosRealesAMover;
      
//       Serial1.print("   Angulo Encoder Actual (antes): "); Serial1.println(anguloEncoderActualAntesDeMover);
//       Serial1.print("   Diferencia Angular Real a cubrir: "); Serial1.println(diferenciaAngularReal);
//       Serial1.print("   Pasos Logicos Reales a mover: "); Serial1.println(pasosRealesAMover);
//       Serial1.print("   Posicion Logica Stepper Actual: "); Serial1.println(motorPasoAPaso.currentPosition());
//       Serial1.print("   Nueva Posicion Logica Stepper Objetivo: "); Serial1.println(nuevaPosicionLogicaStepper);

//       correrStepperHastaPosicion(nuevaPosicionLogicaStepper); // Usamos la nueva posición lógica calculada

//       // --- Verificar con Encoder ---
//       float anguloAbsFinal = encoder.readAngle();
//       float errorAngulo = calcularDiferenciaAngular(anguloAbsFinal, anguloAbsolutoEsperado);
//       Serial1.print("Verificacion Encoder -> Angulo Abs Alcanzado: "); Serial1.print(anguloAbsFinal);
//       Serial1.print(", Error vs Esperado: "); Serial1.print(errorAngulo); Serial1.println(" grados");

//       if (abs(errorAngulo) > TOLERANCIA_ANGULO) {
//         Serial1.println("ADVERTENCIA: Error angular mayor que la tolerancia!");
//       }

//       // --- Esperar 1 Segundo ---
//       Serial1.println("Esperando 1 segundo...");
//       delay(1000);

//       // --- Regresar a la Posición Inicial (anguloEncoderAbsInicial) ---
//       Serial1.println("Regresando a posicion 'Home' logica (0 pasos)...");
//       // Calcular pasos para volver al anguloEncoderAbsInicial desde la posición actual del encoder
//       anguloEncoderActualAntesDeMover = encoder.readAngle(); // Leer de nuevo por si hubo drift
//       diferenciaAngularReal = calcularDiferenciaAngular(anguloEncoderAbsInicial, anguloEncoderActualAntesDeMover);
//       pasosRealesAMover = round(diferenciaAngularReal * PASOS_POR_GRADO_SALIDA);

//       nuevaPosicionLogicaStepper = motorPasoAPaso.currentPosition() + pasosRealesAMover;

//       Serial1.print("   Angulo Encoder Actual (antes de volver): "); Serial1.println(anguloEncoderActualAntesDeMover);
//       Serial1.print("   Diferencia Angular Real a Home: "); Serial1.println(diferenciaAngularReal);
//       Serial1.print("   Pasos Logicos Reales a Home: "); Serial1.println(pasosRealesAMover);
//       Serial1.print("   Nueva Posicion Logica Stepper Objetivo (Home): "); Serial1.println(nuevaPosicionLogicaStepper);
      
//       correrStepperHastaPosicion(nuevaPosicionLogicaStepper); // Mover a la posición lógica calculada para el home

//       // --- Verificar Posición Inicial con Encoder ---
//       float anguloAbsRetorno = encoder.readAngle();
//       float errorRetorno = calcularDiferenciaAngular(anguloAbsRetorno, anguloEncoderAbsInicial); // Comparar con el home original
//       Serial1.print("Verificacion Encoder (Regreso) -> Angulo Abs Alcanzado: "); Serial1.print(anguloAbsRetorno);
//       Serial1.print(", Error respecto al inicio original: "); Serial1.print(errorRetorno); Serial1.println(" grados");

//       // --- Actualizar Estado para el Próximo Ciclo ---
//       anguloEncoderAbsInicial = anguloAbsRetorno; // Re-calibrar el "home" con la última posición real
//       motorPasoAPaso.setCurrentPosition(0); // Resincronizar la posición lógica del stepper a 0
//       posicionLogicaActual = 0;
//       Serial1.print("Posicion logica de pasos resincronizada a 0. Nuevo angulo inicial absoluto de referencia: ");
//       Serial1.println(anguloEncoderAbsInicial);

//       imprimirIndicacion(); // Pedir nueva entrada
//       esperandoEntrada = true; // Volver a esperar entrada

//     } else if (comando != '\n' && comando != '\r') { // Ignorar Enter y CR si no son válidos
//       Serial1.print("Caracter '"); Serial1.print(comando); Serial1.println("' no reconocido.");
//       imprimirIndicacion();
//     }
//   }
//    // motorPasoAPaso.run() no es necesario aquí fuera porque correrStepperHastaPosicion se encarga
// }

//-------------------------------------------------------------------------------------------------------------------------------------
#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h" // La librería AS5600 que te funciona

// --- Constante de Conversión ---
const float RAW_TO_DEGREES = 360.0 / 4096.0; // Para un encoder de 12 bits (2^12 = 4096)

// --- Configuración de Pines ---
const int pinPaso = PA5;
const int pinDireccion = PA6;
// Los pines I2C por defecto en BluePill suelen ser PB7 (SDA) y PB6 (SCL)

// --- Configuración del Motor, Reductor y Encoder ---
const float PASOS_POR_REV_MOTOR = 200.0;      // Pasos por revolución del NEMA 17 (típico 1.8 grados/paso)
const int MICROPASOS = 16;                    // Microstepping configurado en el A4988 (ej: 1, 2, 4, 8, 16)
const float RELACION_REDUCTOR = 30.0;         // Relación de reducción de la base
// Pasos del motor necesarios para una revolución completa del eje de SALIDA
const float PASOS_POR_REV_SALIDA = PASOS_POR_REV_MOTOR * MICROPASOS * RELACION_REDUCTOR;
// Pasos del motor necesarios para girar 90 grados el eje de SALIDA
const long PASOS_PARA_90_GRADOS = round(PASOS_POR_REV_SALIDA / 4.0);
// Pasos por grado en el eje de salida
const float PASOS_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

// --- Configuración de Movimiento (¡AJUSTAR ESTOS VALORES EXPERIMENTALMENTE!) ---
const float VELOCIDAD_MAXIMA = 1500.0 * MICROPASOS; // Velocidad máxima en pasos/segundo (ajusta para tu motor/reductor)
const float ACELERACION = 800.0 * MICROPASOS;       // Aceleración en pasos/segundo^2 (ajusta para suavidad)
const float TOLERANCIA_ANGULO = 1.0;                // Tolerancia en grados para la verificación final

// --- Creación de Objetos ---
// Interfaz DRIVER: Se usa 1 pin para PASO y 1 pin para DIRECCION
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);

// Objeto para el encoder AS5600
AS5600 encoder; // O AS5600L si esa es la clase que te funciona

// --- Variables Globales ---
float anguloEncoderAbsInicial = 0.0; // Ángulo absoluto EN GRADOS medido por el encoder al inicio (o al regresar)
long posicionLogicaActual = 0;       // Posición lógica actual del stepper (en pasos)
bool esperandoEntrada = true;        // Estado para saber si esperamos una 'A' o 'D' (o 'i' y 'd')

// --- Funciones Auxiliares ---

// Normaliza un ángulo para que esté en el rango [0, 360)
float normalizarAngulo(float angulo) {
  angulo = fmod(angulo, 360.0);
  if (angulo < 0) {
    angulo += 360.0;
  }
  return angulo;
}

// Calcula la diferencia más corta entre dos ángulos (resultado entre -180 y +180)
float calcularDiferenciaAngular(float angulo1, float angulo2) {
  float diferencia = normalizarAngulo(angulo1) - normalizarAngulo(angulo2);
  if (diferencia > 180.0) {
    diferencia -= 360.0;
  } else if (diferencia <= -180.0) {
    diferencia += 360.0;
  }
  return diferencia;
}

// Función para leer el ángulo del encoder y convertirlo a grados
float leerAnguloEncoderEnGrados() {
  float rawAngle = encoder.readAngle(); // ASUMO que esto devuelve el valor crudo (0-4095)
  // Si tu librería ya devuelve grados, o si devuelve un error específico (ej. negativo),
  // podrías necesitar ajustar esto o la comprobación de errores.
  // Por ahora, asumimos que devuelve un valor no negativo si la lectura es válida.
  if (rawAngle < 0) { // Un simple chequeo si la lectura cruda da un error como un valor negativo
      Serial1.print("ERROR LEYENDO ENCODER, VALOR CRUDO: ");
      Serial1.println(rawAngle);
      // Podrías querer manejar este error de forma más robusta (ej. detener o reintentar)
      return -1.0; // Devolver un valor de error indicativo
  }
  return rawAngle * RAW_TO_DEGREES;
}


// Función para mover el stepper a la posición deseada (en pasos absolutos lógicos) y esperar
void correrStepperHastaPosicion(long posicionObjetivo) {
    motorPasoAPaso.moveTo(posicionObjetivo);
    Serial1.print("Moviendo stepper a posicion logica: "); Serial1.println(posicionObjetivo);

    while (motorPasoAPaso.distanceToGo() != 0) {
      motorPasoAPaso.run(); // Ejecuta los pasos necesarios
    }
    posicionLogicaActual = motorPasoAPaso.currentPosition(); // Actualizar posición lógica
    Serial1.print("Movimiento stepper completado. Posicion logica final: "); Serial1.println(posicionLogicaActual);
    delay(150); // Pequeña pausa para estabilizar mecánicamente
}

// Función para imprimir el mensaje de solicitud de entrada
void imprimirIndicacion() {
  Serial1.println("------------------------------------");
  Serial1.println("Ingrese 'i' (izquierda 90 grados) o 'd' (derecha 90 grados):");
}


void setup() {
  Serial1.begin(115200); // Usar una velocidad alta es recomendable
  Wire.begin();          // Iniciar comunicación I2C

  Serial1.println("Iniciando Setup...");

  // Configurar pines del motor como salida
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  digitalWrite(pinPaso, LOW);
  digitalWrite(pinDireccion, LOW);

  // Inicializar el encoder AS5600
  encoder.begin();
  // encoder.setDirection(AS5600_CLOCK_WISE); // O AS5600_COUNTER_CLOCK_WISE, ajusta según tu montaje
  
  bool encoderOK = encoder.isConnected(); 
  if (!encoderOK) {
      Serial1.println("ERROR: Encoder AS5600 NO detectado o falla al inicializar.");
      Serial1.println("Verifique conexiones I2C (SDA->PB7, SCL->PB6), VCC (3.3V externo rec.), alimentacion y pull-ups.");
      while(1); // Detener ejecución
  }
  Serial1.println("Encoder AS5600 detectado.");

  // Leer la posición angular inicial ABSOLUTA del encoder EN GRADOS
  anguloEncoderAbsInicial = leerAnguloEncoderEnGrados();
  if (anguloEncoderAbsInicial < 0.0) { // Chequeo si leerAnguloEncoderEnGrados indicó un error
      Serial1.println("ERROR al leer angulo inicial del encoder tras conversion.");
      while(1);
  }
  Serial1.print("Angulo inicial absoluto (GRADOS) medido por Encoder: ");
  Serial1.println(anguloEncoderAbsInicial, 2); // Imprimir con 2 decimales

  // Configuración de AccelStepper
  motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
  motorPasoAPaso.setAcceleration(ACELERACION);
  motorPasoAPaso.setCurrentPosition(0); // Definimos la posición LOGICA actual como 0 PASOS
  posicionLogicaActual = 0;

  Serial1.println("Setup completado.");
  imprimirIndicacion(); // Pedir la primera entrada
}

void loop() {
  // Solo procesar comandos si estamos esperando entrada
  if (esperandoEntrada && Serial1.available() > 0) {
    char comando = Serial1.read();
    comando = tolower(comando); // Usar minúsculas para 'd' e 'i'

    float anguloAbsolutoEsperado = 0.0; // Objetivo angular EN GRADOS
    bool comandoValido = false;

    if (comando == 'd') { // Derecha
      Serial1.println("\nComando 'd' recibido.");
      anguloAbsolutoEsperado = normalizarAngulo(anguloEncoderAbsInicial + 90.0);
      Serial1.print("Moviendo a +90 grados (Objetivo Angulo Abs Esperado GRADOS: "); 
      Serial1.print(anguloAbsolutoEsperado, 2); Serial1.println(")");
      comandoValido = true;

    } else if (comando == 'i') { // Izquierda
      Serial1.println("\nComando 'i' recibido.");
      anguloAbsolutoEsperado = normalizarAngulo(anguloEncoderAbsInicial - 90.0);
      Serial1.print("Moviendo a -90 grados (Objetivo Angulo Abs Esperado GRADOS: "); 
      Serial1.print(anguloAbsolutoEsperado, 2); Serial1.println(")");
      comandoValido = true;
    }

    if (comandoValido) {
      esperandoEntrada = false; // Dejar de esperar mientras se ejecuta

      // --- Ejecutar Movimiento ---
      float anguloEncoderActualAntesDeMover = leerAnguloEncoderEnGrados();
      if (anguloEncoderActualAntesDeMover < 0.0) {
          Serial1.println("Error al leer encoder antes de mover. Abortando movimiento.");
          esperandoEntrada = true; // Volver a esperar
          imprimirIndicacion();
          return; // Salir de esta iteración del loop
      }

      float diferenciaAngularReal = calcularDiferenciaAngular(anguloAbsolutoEsperado, anguloEncoderActualAntesDeMover);
      long pasosRealesAMover = round(diferenciaAngularReal * PASOS_POR_GRADO_SALIDA);
      long nuevaPosicionLogicaStepper = motorPasoAPaso.currentPosition() + pasosRealesAMover;
      
      Serial1.print("   Angulo Encoder Actual (GRADOS antes): "); Serial1.println(anguloEncoderActualAntesDeMover, 2);
      Serial1.print("   Diferencia Angular Real a cubrir (GRADOS): "); Serial1.println(diferenciaAngularReal, 2);
      Serial1.print("   Pasos Logicos Reales a mover: "); Serial1.println(pasosRealesAMover);
      Serial1.print("   Posicion Logica Stepper Actual: "); Serial1.println(motorPasoAPaso.currentPosition());
      Serial1.print("   Nueva Posicion Logica Stepper Objetivo: "); Serial1.println(nuevaPosicionLogicaStepper);

      correrStepperHastaPosicion(nuevaPosicionLogicaStepper);

      // --- Verificar con Encoder ---
      float anguloAbsFinal = leerAnguloEncoderEnGrados();
       if (anguloAbsFinal < 0.0) {
          Serial1.println("Error al leer encoder despues de mover. No se puede verificar.");
          // Continuar con el regreso a home, pero el estado puede ser incierto
      } else {
        float errorAngulo = calcularDiferenciaAngular(anguloAbsFinal, anguloAbsolutoEsperado);
        Serial1.print("Verificacion Encoder -> Angulo Abs Alcanzado (GRADOS): "); Serial1.print(anguloAbsFinal, 2);
        Serial1.print(", Error vs Esperado (GRADOS): "); Serial1.print(errorAngulo, 2); Serial1.println("");

        if (abs(errorAngulo) > TOLERANCIA_ANGULO) {
          Serial1.println("ADVERTENCIA: Error angular mayor que la tolerancia!");
        }
      }

      // --- Esperar 1 Segundo ---
      Serial1.println("Esperando 1 segundo...");
      delay(1000);

      // --- Regresar a la Posición Inicial (anguloEncoderAbsInicial EN GRADOS) ---
      Serial1.println("Regresando a posicion 'Home' (basada en angulo inicial)...");
      
      anguloEncoderActualAntesDeMover = leerAnguloEncoderEnGrados(); // Leer de nuevo por si hubo drift
      if (anguloEncoderActualAntesDeMover < 0.0) {
          Serial1.println("Error al leer encoder antes de regresar a home. Abortando regreso.");
          esperandoEntrada = true; // Volver a esperar
          imprimirIndicacion();
          return; // Salir
      }

      diferenciaAngularReal = calcularDiferenciaAngular(anguloEncoderAbsInicial, anguloEncoderActualAntesDeMover);
      pasosRealesAMover = round(diferenciaAngularReal * PASOS_POR_GRADO_SALIDA);
      nuevaPosicionLogicaStepper = motorPasoAPaso.currentPosition() + pasosRealesAMover;

      Serial1.print("   Angulo Encoder Actual (GRADOS antes de volver): "); Serial1.println(anguloEncoderActualAntesDeMover, 2);
      Serial1.print("   Diferencia Angular Real a Home (GRADOS): "); Serial1.println(diferenciaAngularReal, 2);
      Serial1.print("   Pasos Logicos Reales a Home: "); Serial1.println(pasosRealesAMover);
      Serial1.print("   Nueva Posicion Logica Stepper Objetivo (Home): "); Serial1.println(nuevaPosicionLogicaStepper);
      
      correrStepperHastaPosicion(nuevaPosicionLogicaStepper);

      // --- Verificar Posición Inicial con Encoder ---
      float anguloAbsRetorno = leerAnguloEncoderEnGrados();
      if (anguloAbsRetorno < 0.0) {
          Serial1.println("Error al leer encoder despues de regresar a home.");
          // El anguloEncoderAbsInicial no se actualizará con un valor erróneo
      } else {
        float errorRetorno = calcularDiferenciaAngular(anguloAbsRetorno, anguloEncoderAbsInicial);
        Serial1.print("Verificacion Encoder (Regreso) -> Angulo Abs Alcanzado (GRADOS): "); Serial1.print(anguloAbsRetorno, 2);
        Serial1.print(", Error respecto al inicio original (GRADOS): "); Serial1.print(errorRetorno, 2); Serial1.println("");
        
        // --- Actualizar Estado para el Próximo Ciclo ---
        // Re-calibrar el "home" con la última posición real en grados
        anguloEncoderAbsInicial = anguloAbsRetorno; 
      }
      
      motorPasoAPaso.setCurrentPosition(0); // Resincronizar la posición lógica del stepper a 0
      posicionLogicaActual = 0;
      Serial1.print("Posicion logica de pasos resincronizada a 0. Nuevo angulo inicial de referencia (GRADOS): ");
      Serial1.println(anguloEncoderAbsInicial, 2);

      imprimirIndicacion(); // Pedir nueva entrada
      esperandoEntrada = true; // Volver a esperar entrada

    } else if (comando != '\n' && comando != '\r') { // Ignorar Enter y CR si no son válidos
      Serial1.print("Caracter '"); Serial1.print(comando); Serial1.println("' no reconocido.");
      imprimirIndicacion();
    }
  }
}