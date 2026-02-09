#include <Arduino.h>

/**
 * PROYECTO: Brazo Robótico - Búsqueda de Home Direccional con Ajuste Fino
 * LOGICA MODIFICADA:
 * 1. Inicio Manual: Mover motor con 'a'/'d' a posicion de salida.
 * 2. 'R' (Ready): Inicia búsqueda.
 * 3. Búsqueda Inicial:
 *    - Si Sensor es NEGRO -> Gira DERECHA buscando BLANCO.
 *    - Si Sensor es BLANCO -> Gira IZQUIERDA buscando NEGRO.
 * 4. Transición Detectada -> Inicia "Barrido Oscilatorio Local".
 * 5. Barrido Local: Pequeña oscilación para confirmar/precisar el Home.
 */

// --- PINOUT ---
#define PIN_SENSOR_ANALOG PA0
#define PIN_STEP          PA5
#define PIN_DIR           PA6

// --- DEFINICIÓN DE ZONAS ---
const int LIMITE_BLANCO = 200;  
const int LIMITE_NEGRO  = 600;  

// --- VELOCIDADES ---
const int VEL_CRUCERO = 800;    
const int VEL_APROX   = 2500;  

// --- PARÁMETROS DE BÚSQUEDA ---
const long AMPLITUD_INICIAL = 1000;     // Usado solo como fallback en oscilación
const long RANGO_OSCILACION_FINA = 400; // Rango para el barrido local tras transición
const long INCREMENTO_AMPLITUD = 1000; 

// Estados
enum EstadoMovimiento { PARADO, MOVIENDO_DERECHA, MOVIENDO_IZQUIERDA };
EstadoMovimiento estado_motor = PARADO;

// Variables Globales
bool sistema_listo = false;
int velocidad_actual = VEL_CRUCERO;

// Variables para Algoritmo de Búsqueda
long posicion_relativa = 0;       
long limite_actual_busqueda = AMPLITUD_INICIAL;
bool buscando_negro = true;       
bool fase_aproximacion = true; // TRUE = Busqueda direccional, FALSE = Oscilacion fina

void setup() {
  Serial2.begin(115200);
 
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_SENSOR_ANALOG, INPUT_ANALOG);

  Serial2.println("--- SISTEMA DE BUSQUEDA DIRECCIONAL + OSCILACION ---");
  Serial2.println("1. Use 'a'/'d' para posicionar el motor.");
  Serial2.println("2. Presione 'R' para iniciar la busqueda del Home.");
}

void loop() {
  // 1. Lectura del Sensor
  int lectura = analogRead(PIN_SENSOR_ANALOG);
 
  // Imprimir estado
  static long ultima_impresion = 0;
  if (millis() - ultima_impresion > 500) {
    Serial2.print("Sensor: "); Serial2.print(lectura);
    Serial2.print(" | PosRel: "); Serial2.print(posicion_relativa);
    if (sistema_listo) {
        if (fase_aproximacion) Serial2.print(" [APROXIMANDO]");
        else {
            Serial2.print(" | Rango: +/-"); Serial2.print(limite_actual_busqueda);
            Serial2.print(" [SINTONIA FINA]");
        }
    } else {
        Serial2.print(" [MANUAL]");
    }
    Serial2.println();
    ultima_impresion = millis();
  }

  // 2. Gestión de Comandos
  if (Serial2.available()) {
    char cmd = Serial2.read();
   
    // START (R): Iniciar algoritmo
    if (!sistema_listo && (cmd == 'R' || cmd == 'r')) {
      sistema_listo = true;
      posicion_relativa = 0;
      fase_aproximacion = true; // Empezamos en modo direccional
      limite_actual_busqueda = AMPLITUD_INICIAL; // Reset de seguridad
     
      // Lógica de inicio según sensor
      if (lectura < 400) {
        // Estamos en BLANCO -> Buscamos NEGRO -> Girar IZQUIERDA
        buscando_negro = true;
        estado_motor = MOVIENDO_IZQUIERDA;
        digitalWrite(PIN_DIR, LOW);
        Serial2.println(">>> INICIO (En Blanco) -> IZQUIERDA buscando NEGRO");
      } else {
        // Estamos en NEGRO (o gris alto) -> Buscamos BLANCO -> Girar DERECHA
        buscando_negro = false;
        estado_motor = MOVIENDO_DERECHA;
        digitalWrite(PIN_DIR, HIGH);
        Serial2.println(">>> INICIO (En Negro) -> DERECHA buscando BLANCO");
      }
    }
    else if (cmd == 'a') { // Manual Izq
      if(!sistema_listo) {
        estado_motor = MOVIENDO_IZQUIERDA;
        digitalWrite(PIN_DIR, LOW);
      }
    }
    else if (cmd == 'd') { // Manual Der
      if(!sistema_listo) {
        estado_motor = MOVIENDO_DERECHA;
        digitalWrite(PIN_DIR, HIGH);
      }
    }
    else if (cmd == 's') { // Stop
      estado_motor = PARADO;
      sistema_listo = false;
      Serial2.println("|| STOP");
    }
  }

  // 3. Lógica de Control
  if (estado_motor != PARADO) {
   
    if (sistema_listo) {
      
      // A. Verificar si encontramos el OBJETIVO
      bool objetivo_encontrado = false;
      if (buscando_negro && lectura >= LIMITE_NEGRO) objetivo_encontrado = true;
      if (!buscando_negro && lectura <= LIMITE_BLANCO) objetivo_encontrado = true;
     
      if (objetivo_encontrado) {
        if (fase_aproximacion) {
            // Terminó la fase de aproximación, pasamos a sintonía fina
            fase_aproximacion = false;
            posicion_relativa = 0; // Resetear posición local
            limite_actual_busqueda = RANGO_OSCILACION_FINA; 
            
            // Invertimos la búsqueda para la oscilación
            // (Si encontramos Negro, ahora buscamos Blanco para cruzar de nuevo)
            buscando_negro = !buscando_negro;
            
            // Invertimos dirección física para volver a cruzar la transición
            if (estado_motor == MOVIENDO_DERECHA) {
                estado_motor = MOVIENDO_IZQUIERDA;
                digitalWrite(PIN_DIR, LOW);
            } else {
                estado_motor = MOVIENDO_DERECHA;
                digitalWrite(PIN_DIR, HIGH);
            }

            Serial2.println(">>> TRANSICION DETECTADA -> Iniciando Barrido Local");
        } 
        else {
            // Si ya estamos en fase fina y encontramos el objetivo (de nuevo), es el HOME definitivo
            estado_motor = PARADO;
            Serial2.print("!!! HOME CONFIRMADO EN: ");
            Serial2.println(posicion_relativa);
            return; 
        }
      }

      // B. Ajustar Velocidad
      if (lectura >= LIMITE_BLANCO && lectura < LIMITE_NEGRO) {
        velocidad_actual = VEL_APROX;
      } else {
        velocidad_actual = VEL_CRUCERO;
      }

      // C. Lógica de Movimiento (Diferente por fase)
      if (!fase_aproximacion) {
          // --- MODO OSCILATORIO (Solo en Fase Fina) ---
          // Si superamos el límite sin encontrar nada, invertimos y expandimos
          
          if (estado_motor == MOVIENDO_DERECHA && posicion_relativa >= limite_actual_busqueda) {
            estado_motor = MOVIENDO_IZQUIERDA;
            digitalWrite(PIN_DIR, LOW);
            Serial2.println("<< FINA: Cambio a IZQ");
          }
          else if (estado_motor == MOVIENDO_IZQUIERDA && posicion_relativa <= -limite_actual_busqueda) {
            estado_motor = MOVIENDO_DERECHA;
            digitalWrite(PIN_DIR, HIGH);
            // Expandir si falla la fina (failsafe)
            limite_actual_busqueda += 500; 
            Serial2.println(">> FINA: Cambio a DER y EXPANSION");
          }
      }
      // En fase_aproximacion NO hay limites de distancia, solo avanza hasta encontrar.
    }
    else {
      // Manual
      velocidad_actual = VEL_CRUCERO;
      posicion_relativa = 0;
    }

    // 4. Ejecución Física del Paso
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(velocidad_actual);

    // 5. Actualizar posición relativa
    if (sistema_listo) {
      if (estado_motor == MOVIENDO_DERECHA) posicion_relativa++;
      else if (estado_motor == MOVIENDO_IZQUIERDA) posicion_relativa--;
    }
  }
}