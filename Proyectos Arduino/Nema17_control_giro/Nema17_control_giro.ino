#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h" // La librería AS5600 que te funciona

// --- Constante de Conversión ---
const float RAW_A_GRADOS = 360.0 / 4096.0;

// --- Configuración de Pines ---
const int pinPaso = PA5;
const int pinDireccion = PA6;

// --- Configuración del Motor, Reductor y Encoder ---
const float PASOS_POR_REV_MOTOR = 200.0;
const int MICROPASOS = 16; // ¡¡VERIFICAR CONFIGURACIÓN FÍSICA DEL A4988!!
const float RELACION_REDUCTOR = 1.0; // SIN REDUCTOR
const float PASOS_POR_REV_SALIDA = PASOS_POR_REV_MOTOR * MICROPASOS * RELACION_REDUCTOR;
const float PASOS_POR_GRADO_SALIDA = PASOS_POR_REV_SALIDA / 360.0;

// --- Configuración de Movimiento (¡AJUSTAR!) ---
// Empezar con valores mas bajos
 const float VELOCIDAD_MAXIMA = 800.0 * MICROPASOS;
const float ACELERACION = 400.0 * MICROPASOS;
const float TOLERANCIA_ANGULO = 1.0;

// --- Objetos ---
AccelStepper motorPasoAPaso(AccelStepper::DRIVER, pinPaso, pinDireccion);
AS5600 encoder;

// --- Variables Globales ---
float anguloEncoderAbsHome = 0.0; // Ángulo absoluto EN GRADOS que define nuestro "Home"
long posicionLogicaActual = 0;    // Posición actual del stepper según AccelStepper (en micropasos).
bool esperandoEntrada = false;    // Se pondrá a true DESPUÉS de la calibración inicial
bool primerMovimientoHecho = false; // Bandera para la lógica de la máquina de estados en loop()
                                    // para saber si ya se hizo el movimiento hacia +/-90 grados.
char ultimoComandoChar = ' ';     // Guarda el último comando ('d' o 'i') para la lógica de retorno.

// --- Funciones Auxiliares ---
float normalizarAngulo(float angulo) {      //Toma un ángulo y lo devuelve en el rango [0, 360) grados.
  angulo = fmod(angulo, 360.0);             //Ej: 370 -> 10, -10 -> 350
  if (angulo < 0) {
    angulo += 360.0;
  }
  return angulo;
}

float calcularDiferenciaAngular(float anguloObjetivo, float anguloActual) {                 //Calcula la diferencia angular más corta (con signo)
  float diferencia = normalizarAngulo(anguloObjetivo) - normalizarAngulo(anguloActual);     //entre dos ángulos. El resultado estará entre -180 y +180 grados.
  if (diferencia > 180.0) {                                                                 //Ej: angulo1=10, angulo2=350. Diferencia directa = -340. Diferencia más corta = +20.
    diferencia -= 360.0;
  } else if (diferencia <= -180.0) {
    diferencia += 360.0;
  }
  return diferencia;
}

float leerAnguloEncoderEnGrados() {
  word rawAngle = encoder.readAngle(); // Asumo que esto devuelve el valor crudo (0-4095)
  return (float)rawAngle * RAW_A_GRADOS;
}

void correrStepperHastaPosicion(long posicionObjetivoLogico) {      //Mueve el motor a una 'posicionObjetivoLogico' (en micropasos) y espera hasta que el movimiento se complete.
    motorPasoAPaso.moveTo(posicionObjetivoLogico);                  //Le dice a AccelStepper el nuevo objetivo.
    Serial1.print("Moviendo stepper a posicion logica: "); Serial1.println(posicionObjetivoLogico);
    while (motorPasoAPaso.distanceToGo() != 0) {
      motorPasoAPaso.run();
    }
    Serial1.print("Movimiento stepper completado. Posicion logica final: "); Serial1.println(motorPasoAPaso.currentPosition());
    delay(200); // Pausa para estabilizar
}

void imprimirIndicacion() {
  Serial1.println("------------------------------------");
  Serial1.println("Ingrese 'i' (izquierda 90) o 'd' (derecha 90):");
}

void setup() {
  Serial1.begin(115200);    // Inicia comunicación serie con el PC (Monitor Serie).
  Wire.begin();             // Inicia comunicación I2C para el encoder.
  Serial1.println("Iniciando Setup (SIN REDUCTOR)...");
  Serial1.print("PASOS_POR_GRADO_SALIDA (calculado): "); Serial1.println(PASOS_POR_GRADO_SALIDA, 4);

  // Configurar pines del motor como SALIDA.
  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  digitalWrite(pinPaso, LOW);     // Estado inicial seguro.
  digitalWrite(pinDireccion, LOW);

  // Inicializar el encoder AS5600.
  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE); // Configura la dirección de aumento del ángulo.

  bool encoderOK = encoder.isConnected(); // Verifica si el encoder responde.
  
  if (!encoderOK) {
      Serial1.println("ERROR: Encoder AS5600 NO detectado.");
      while(1); // Detiene el programa si no hay encoder.
  }
  Serial1.println("Encoder AS5600 detectado.");
  delay(500);

  // --- CALIBRACIÓN INICIAL A 0 GRADOS (CONCEPTUAL) ---
  Serial1.println("Calibrando a posicion CERO...");
  float anguloLecturaInicial = leerAnguloEncoderEnGrados();               // Lee la posición física de arranque.
  if (anguloLecturaInicial < 0.0) {                                       // Asumiendo que -1 es error
      Serial1.println("ERROR al leer angulo para calibracion inicial.");
      while(1);
  }
  Serial1.print("Angulo de lectura inicial (GRADOS): "); Serial1.println(anguloLecturaInicial, 2);

  // El "0 grados conceptual" será el ángulo más cercano a 0, 360, 720 etc.
  // Para simplificar, llevaremos la lectura actual del encoder a lo que consideraremos 0 grados.
  // Si el ángulo actual es, por ejemplo, 20 grados, y queremos que esa posición sea 0,
  // necesitamos movernos -20 grados.
  // Si el ángulo actual es 350, y queremos que esa posición sea 0 (o 360 que es lo mismo),
  // necesitamos movernos +10 grados.

  float anguloObjetivoCalibracion = 0.0; // Nuestro 0 grados conceptual
  
  // Calcula cuánto hay que moverse para que la 'anguloLecturaInicial' se convierta en 'anguloObjetivoCalibracion'.
  float diferenciaParaCalibrar = calcularDiferenciaAngular(anguloObjetivoCalibracion, anguloLecturaInicial);
  
  Serial1.print("Diferencia para calibrar a CERO (GRADOS): "); Serial1.println(diferenciaParaCalibrar, 2);
 
  long pasosParaCalibrar = round(diferenciaParaCalibrar * PASOS_POR_GRADO_SALIDA);  // Convierte a pasos.
  Serial1.print("Pasos para calibrar: "); Serial1.println(pasosParaCalibrar);

  motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA / 2); // Usar velocidad más lenta para calibrar
  motorPasoAPaso.setAcceleration(ACELERACION / 2); // Usar aceleración más lenta
  motorPasoAPaso.setCurrentPosition(0); // El stepper está lógicamente en 0
  
  correrStepperHastaPosicion(pasosParaCalibrar); // Mover los pasos calculados

  delay(500); // Esperar que se asiente
  anguloEncoderAbsHome = leerAnguloEncoderEnGrados(); // Lee la posición física final después de calibrar. Este es nuestro nuevo HOME absoluto
  if (anguloEncoderAbsHome < 0.0) {
      Serial1.println("ERROR al leer angulo para definir HOME.");
      while(1);
  }
  // Forzar que el "home" sea lo más cercano a 0 o 360 posible después del movimiento.
  // Esto es para que los siguientes cálculos de +/- 90 sean más intuitivos.
  if (abs(calcularDiferenciaAngular(anguloEncoderAbsHome, 0.0)) > abs(calcularDiferenciaAngular(anguloEncoderAbsHome, 360.0))) {
      // Si está más cerca de 360 (ej. 358), lo consideramos 0 para la próxima normalización
      // Esto se maneja bien con normalizeAngle y calculateDifferenceAngular si anguloEncoderAbsHome se usa consistentemente.
      // No es estrictamente necesario hacer anguloEncoderAbsHome = 0.0 aquí si las funciones de ángulo son robustas.
      // Pero para claridad de los logs:
      if (calcularDiferenciaAngular(anguloEncoderAbsHome, 0.0) < -170) anguloEncoderAbsHome = 0.0 + (anguloEncoderAbsHome - 360.0);
      else if (calcularDiferenciaAngular(anguloEncoderAbsHome, 0.0) > 170) anguloEncoderAbsHome = 0.0 + (anguloEncoderAbsHome + 360.0);
      // Lo más simple es normalizarlo después de leer:
      anguloEncoderAbsHome = normalizarAngulo(anguloEncoderAbsHome);
      if (anguloEncoderAbsHome > 180) anguloEncoderAbsHome -= 360; // Preferir rango -180 a 180 para el home si está cerca de los extremos
      // Para el propósito de la lógica, el anguloEncoderAbsHome es la lectura *real*
  }


  motorPasoAPaso.setCurrentPosition(0); // Después de calibrar, esta posición física es el 0 lógico del stepper.
  posicionLogicaActual = 0;

  Serial1.print("Calibracion completada. Angulo Home absoluto (GRADOS) Encoder: "); Serial1.println(anguloEncoderAbsHome, 2);
  Serial1.println("Posicion Logica Stepper establecida a 0.");
  
  // Restaurar velocidad y aceleración normales para operación
  motorPasoAPaso.setMaxSpeed(VELOCIDAD_MAXIMA);
  motorPasoAPaso.setAcceleration(ACELERACION);
  
  esperandoEntrada = true; // Ahora sí, esperar comandos del usuario
  imprimirIndicacion();
}

void loop() {
  if (esperandoEntrada && Serial1.available() > 0) {
    char comando = Serial1.read();
    comando = tolower(comando);

    float anguloObjetivoRelativoGrados = 0.0;
    bool comandoValido = false;

    float anguloEncoderActualParaCalculo = leerAnguloEncoderEnGrados();
     if (anguloEncoderActualParaCalculo < -900.0 ) { // Error en lectura
        Serial1.println("Error al leer encoder antes de procesar comando. Ignorando comando.");
        imprimirIndicacion();
        return;
    }

    if (comando == 'd') {
      Serial1.println("\nComando 'd' recibido.");
      anguloObjetivoRelativoGrados = 90.0;
      comandoValido = true;
      ultimoComandoChar = 'd';
    } else if (comando == 'i') {
      Serial1.println("\nComando 'i' recibido.");
      anguloObjetivoRelativoGrados = -90.0;
      comandoValido = true;
      ultimoComandoChar = 'i';
    }

    if (comandoValido) {
      esperandoEntrada = false;
      primerMovimientoHecho = false;

      float anguloEncoderObjetivoAbs = normalizarAngulo(anguloEncoderAbsHome + anguloObjetivoRelativoGrados);
      Serial1.print("Moviendo a "); Serial1.print(anguloObjetivoRelativoGrados);
      Serial1.print(" grados rel. (Obj. Encoder Abs: "); Serial1.print(anguloEncoderObjetivoAbs, 2); Serial1.println(")");

      float diferenciaGradosRequerida = calcularDiferenciaAngular(anguloEncoderObjetivoAbs, anguloEncoderActualParaCalculo);
      long pasosAMover = round(diferenciaGradosRequerida * PASOS_POR_GRADO_SALIDA);
      long nuevaPosicionLogicaObjetivo = motorPasoAPaso.currentPosition() + pasosAMover;
      
      Serial1.print("   Ang Encoder Actual (GRADOS): "); Serial1.println(anguloEncoderActualParaCalculo, 2);
      Serial1.print("   Dif Angular a cubrir (GRADOS): "); Serial1.println(diferenciaGradosRequerida, 2);
      Serial1.print("   Pasos Logicos a mover: "); Serial1.println(pasosAMover);
      
      correrStepperHastaPosicion(nuevaPosicionLogicaObjetivo);
      primerMovimientoHecho = true;
    
    } else if (comando != '\n' && comando != '\r') {
      Serial1.print("Caracter '"); Serial1.print(comando); Serial1.println("' no reconocido.");
      imprimirIndicacion();
    }
  } 

  if (primerMovimientoHecho && motorPasoAPaso.distanceToGo() == 0) {
      if (motorPasoAPaso.currentPosition() != 0) { 
          Serial1.println("   Llego al objetivo (+/-90 grados).");
          float anguloAlLlegar = leerAnguloEncoderEnGrados();
          if (anguloAlLlegar >= -360.0) { // Chequeo básico de validez
            float anguloObjetivoAbsCalculado = normalizarAngulo(anguloEncoderAbsHome + (ultimoComandoChar == 'd' ? 90.0 : -90.0));
            float errorReal = calcularDiferenciaAngular(anguloAlLlegar, anguloObjetivoAbsCalculado);
            Serial1.print("   Verificacion Encoder -> Angulo Abs Alcanzado (GRADOS): "); Serial1.print(anguloAlLlegar, 2);
            Serial1.print(", Error vs Esperado (GRADOS): "); Serial1.println(errorReal, 2);
            if (abs(errorReal) > TOLERANCIA_ANGULO) Serial1.println("   ADVERTENCIA: Error angular grande!");
          } else { Serial1.println("   Error leyendo encoder en objetivo."); }

          Serial1.println("   Esperando 1 segundo...");
          delay(1000);

          Serial1.println("   Regresando a posicion 'Home' (logica 0)...");
          correrStepperHastaPosicion(0); 

      } else { 
          Serial1.println("   Llego de vuelta al 'Home' logico.");
          float anguloEnHome = leerAnguloEncoderEnGrados();
           if (anguloEnHome >= -360.0) { // Chequeo básico
            float errorHome = calcularDiferenciaAngular(anguloEnHome, anguloEncoderAbsHome);
            Serial1.print("   Verificacion Encoder (Regreso) -> Angulo Abs Alcanzado (GRADOS): "); Serial1.print(anguloEnHome, 2);
            Serial1.print(", Error respecto al Home original (GRADOS): "); Serial1.println(errorHome, 2);
            
            // Re-calibrar el Home para el próximo ciclo de comando completo
            anguloEncoderAbsHome = anguloEnHome; 
          } else { Serial1.println("   Error leyendo encoder al regresar a Home."); }
          
          motorPasoAPaso.setCurrentPosition(0); // Re-sincronizar AccelStepper a CERO lógico
          posicionLogicaActual = 0; // Actualizar nuestra variable de posición lógica
          Serial1.print("   Posicion logica resincronizada. Nuevo angulo Home (GRADOS): "); Serial1.println(anguloEncoderAbsHome, 2);
          
          primerMovimientoHecho = false; 
          esperandoEntrada = true;       
          ultimoComandoChar = ' ';
          imprimirIndicacion();
      }
  }

  if (!esperandoEntrada || motorPasoAPaso.distanceToGo() != 0) {
      motorPasoAPaso.run();
  }
}