#include <Wire.h>
#include "AS5600.h" // Tu librería AS5600
#include <AccelStepper.h> // Aunque no la usemos para el perfil, la dejamos por si acaso para futuras pruebas

// --- Constante de Conversión ---
const float RAW_A_GRADOS = 360.0 / 4096.0;

// --- Configuración de Pines ---
const int pinPaso = PA5;
const int pinDireccion = PA6;

// --- Configuración del Motor, Reductor y Encoder ---
const float PASOS_POR_REV_MOTOR_COMPLETO = 200.0;
const int MICROPASOS_CONFIGURADOS = 16;
const float RELACION_REDUCTOR = 37; // ¡CON REDUCTOR!
const float PASOS_MOTOR_POR_REV_EFECTIVO = PASOS_POR_REV_MOTOR_COMPLETO * MICROPASOS_CONFIGURADOS;
const float PASOS_POR_REV_EJE_SALIDA = PASOS_MOTOR_POR_REV_EFECTIVO * RELACION_REDUCTOR;
const float PASOS_MOTOR_POR_GRADO_EJE_SALIDA = PASOS_POR_REV_EJE_SALIDA / 360.0;

// --- Objeto Encoder ---
AS5600 encoder;

// --- Variables Globales para el Perfil (se usarán en ejecutarPerfilTriangular) ---
long q_ini_logico_pasos_perfil = 0; // No se usa realmente si el movimiento es siempre relativo al inicio del perfil
long q_fin_logico_pasos_perfil = 0;
float V_pico_pasos_por_seg_perfil = 0;
float T_total_seg_perfil = 0;
float aceleracion_calculada_perfil = 0;

// --- Variables de Estado y Medición ---
unsigned long tiempo_inicio_movimiento_micros_perfil = 0;
unsigned long tiempo_fin_movimiento_micros_perfil = 0;
long pasos_reales_dados_en_segmento_perfil = 0;
long encoder_antes_mov_perfil = 0;


// --- Funciones Auxiliares ---
float normalizarAngulo(float angulo) {
  angulo = fmod(angulo, 360.0);
  if (angulo < 0) {
    angulo += 360.0;
  }
  return angulo;
}

float calcularDiferenciaAngular(float anguloFinal, float anguloInicial) {
  float diferencia = normalizarAngulo(anguloFinal) - normalizarAngulo(anguloInicial);
  if (diferencia > 180.0) {
    diferencia -= 360.0; // Tomar el camino más corto por el otro lado
  } else if (diferencia < -180.0) { // Nota: era <= -180.0, pero < -180.0 es más simétrico
    diferencia += 360.0;
  }
  return diferencia;
}

float leerAnguloEncoderEnGrados() {
  word rawAngle = encoder.readAngle();
  return (float)rawAngle * RAW_A_GRADOS;
}

long leerPasosCrudosEncoder() {
  return encoder.readAngle();
}

//---------------------------
void setup() {
  Serial1.begin(115200);
  Wire.begin();

  Serial1.println("--- Setup Perfil Triangular Motor (CON REDUCTOR, Encoder en EJE MOTOR) ---");
  Serial1.print("PASOS_MOTOR_POR_GRADO_EJE_SALIDA (calculado): "); Serial1.println(PASOS_MOTOR_POR_GRADO_EJE_SALIDA, 4);

  pinMode(pinPaso, OUTPUT);
  pinMode(pinDireccion, OUTPUT);
  digitalWrite(pinPaso, LOW);
  digitalWrite(pinDireccion, LOW);

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE); // Asegúrate que esta sea la dirección correcta
  if (!encoder.isConnected()) {
    Serial1.println("ERROR: Encoder NO detectado!");
    while (1);
  }
  Serial1.println("Encoder detectado.");
  delay(500);

  float angulo_motor_inicial_grados = leerAnguloEncoderEnGrados(); // Lectura del eje del motor
  Serial1.print("Angulo inicial EJE MOTOR (GRADOS) Encoder al arrancar: "); Serial1.println(angulo_motor_inicial_grados, 2);

  Serial1.println("Setup completado. Iniciando perfil triangular automaticamente...");
  Serial1.println("------------------------------------------------------");

  // --- EJECUCIÓN AUTOMÁTICA DEL PERFIL ---
  // Definir los parámetros para un segmento corto EN EL EJE DE SALIDA
  float grados_eje_salida_objetivo = 15.0; // Mover 15 grados el eje de SALIDA del reductor
  float vel_pico_eje_salida_grados_por_seg = 30.0; // Pico de velocidad de 30 grados/seg en la SALIDA

  // Convertir a parámetros del MOTOR (estos son los que usa ejecutarPerfilTriangular)
  long pasos_motor_objetivo_netos = round(grados_eje_salida_objetivo * PASOS_MOTOR_POR_GRADO_EJE_SALIDA);
  float vel_pico_motor_pps = vel_pico_eje_salida_grados_por_seg * PASOS_MOTOR_POR_GRADO_EJE_SALIDA;

  // Ejecutar perfil hacia adelante
  Serial1.print("Objetivo EJE SALIDA: +"); Serial1.print(grados_eje_salida_objetivo, 2); Serial1.println(" grados");
  ejecutarPerfilTriangular(pasos_motor_objetivo_netos, vel_pico_motor_pps, true); // true para CW (derecha)
  
  delay(2000); // Esperar 2 segundos

  // Ejecutar perfil de regreso (misma magnitud, dirección opuesta)
  Serial1.print("\nIniciando perfil de regreso. Objetivo EJE SALIDA: -"); Serial1.print(grados_eje_salida_objetivo, 2); Serial1.println(" grados (para volver al inicio)");
  ejecutarPerfilTriangular(pasos_motor_objetivo_netos, vel_pico_motor_pps, false); // false para CCW (izquierda)
  
  Serial1.println("\n--- Demostracion de perfiles completada ---");
}

// La función ejecutarPerfilTriangular ahora toma pasos_MOTOR_objetivo_netos
// y velocidad_pico_MOTOR_pps. El signo de pasos_MOTOR_objetivo_netos ya no se usa
// para la dirección, se pasa explícitamente en 'direccion_fisica_cw'.
void ejecutarPerfilTriangular(long pasos_motor_a_dar_magnitud, float velocidad_pico_motor_pps, bool direccion_fisica_cw) {
  if (pasos_motor_a_dar_magnitud == 0) {
    Serial1.println("Pasos objetivo es 0, no hay movimiento.");
    return;
  }

  Serial1.println("\n--- Iniciando Segmento de Perfil Triangular ---");
  q_fin_logico_pasos_perfil = pasos_motor_a_dar_magnitud; // Magnitud de pasos del motor

  T_total_seg_perfil = (2.0 * q_fin_logico_pasos_perfil) / velocidad_pico_motor_pps;
  if (T_total_seg_perfil <= 0.001) {
      Serial1.println("Error: T_total_perfil calculado es muy pequeño o <=0.");
      return;
  }
  aceleracion_calculada_perfil = (2.0 * velocidad_pico_motor_pps) / T_total_seg_perfil;
  if (aceleracion_calculada_perfil <= 0.0){ // La aceleración debe ser positiva
    Serial1.println("Error: Aceleracion calculada es <= 0.");
    return;
  }
  
  float t_pico_seg = T_total_seg_perfil / 2.0;
  // Los pasos hasta el pico deberían ser la mitad de los pasos totales del motor para un perfil simétrico
  long pasos_motor_hasta_pico = q_fin_logico_pasos_perfil / 2;
  long pasos_motor_fase_decel = q_fin_logico_pasos_perfil - pasos_motor_hasta_pico;


  Serial1.print("Parametros Motor: Pasos_Comandados_Magnitud="); Serial1.print(q_fin_logico_pasos_perfil);
  Serial1.print(", V_pico_Motor (pps)="); Serial1.print(velocidad_pico_motor_pps, 2);
  Serial1.print(", T_total_Calculado (s)="); Serial1.println(T_total_seg_perfil, 3);
  Serial1.print("Aceleracion_Motor_Calculada (pps^2): "); Serial1.println(aceleracion_calculada_perfil, 2);
  Serial1.print("Pasos_Motor_Hasta_Pico: "); Serial1.println(pasos_motor_hasta_pico);

  if (direccion_fisica_cw) {
    digitalWrite(pinDireccion, HIGH); Serial1.println("Direccion Fisica Motor: CW (HIGH)");
  } else {
    digitalWrite(pinDireccion, LOW);  Serial1.println("Direccion Fisica Motor: CCW (LOW)");
  }

  float tiempo_actual_seg = 0;
  float tiempo_paso_anterior_seg = 0;
  unsigned long retardo_entre_pulsos_micros;
  pasos_reales_dados_en_segmento_perfil = 0;

  encoder_antes_mov_perfil = leerPasosCrudosEncoder();
  tiempo_inicio_movimiento_micros_perfil = micros();

  // --- Fase de Aceleración del MOTOR ---
  Serial1.println("Iniciando Fase de Aceleracion Motor...");
  for (long i = 1; i <= pasos_motor_hasta_pico; i++) {
    tiempo_actual_seg = sqrt(2.0 * i / aceleracion_calculada_perfil);
    retardo_entre_pulsos_micros = (tiempo_actual_seg - tiempo_paso_anterior_seg) * 1000000.0;
    unsigned long retardo_pulso = retardo_entre_pulsos_micros / 2;
    if (retardo_pulso < 50) retardo_pulso = 50; // Límite inferior de retardo

    digitalWrite(pinPaso, HIGH); delayMicroseconds(retardo_pulso);
    digitalWrite(pinPaso, LOW);  delayMicroseconds(retardo_pulso);
    
    tiempo_paso_anterior_seg = tiempo_actual_seg;
    pasos_reales_dados_en_segmento_perfil++;
  }
  Serial1.println("Fin Fase de Aceleracion Motor.");

  // --- Fase de Deceleración del MOTOR ---
  Serial1.println("Iniciando Fase de Deceleracion Motor...");
  for (long i = (pasos_motor_fase_decel -1) ; i >= 0; i--) {
      float tiempo_objetivo_para_este_paso_en_fase_decel = t_pico_seg + (t_pico_seg - sqrt(2.0 * i / aceleracion_calculada_perfil));
      retardo_entre_pulsos_micros = (tiempo_objetivo_para_este_paso_en_fase_decel - tiempo_paso_anterior_seg) * 1000000.0;
      unsigned long retardo_pulso = retardo_entre_pulsos_micros / 2;
      if (retardo_pulso < 50) retardo_pulso = 50;

      digitalWrite(pinPaso, HIGH); delayMicroseconds(retardo_pulso);
      digitalWrite(pinPaso, LOW);  delayMicroseconds(retardo_pulso);

      tiempo_paso_anterior_seg = tiempo_objetivo_para_este_paso_en_fase_decel;
      pasos_reales_dados_en_segmento_perfil++;
  }
  Serial1.println("Fin Fase de Deceleracion Motor.");
  
  tiempo_fin_movimiento_micros_perfil = micros();

  // --- Reporte Final del Segmento ---
  long encoder_despues_mov = leerPasosCrudosEncoder();
  long variacion_encoder_crudo_motor = encoder_despues_mov - encoder_antes_mov_perfil;

  // Lógica de Wraparound para el encoder del MOTOR
  // AS5600_CLOCK_WISE significa que el ángulo del encoder AUMENTA con giro físico CW del motor.
  if (direccion_fisica_cw) { 
      if (variacion_encoder_crudo_motor < -2048) { variacion_encoder_crudo_motor += 4096; }
  } else { 
      if (variacion_encoder_crudo_motor > 2048) { variacion_encoder_crudo_motor -= 4096; }
  }
  
  if (!direccion_fisica_cw && variacion_encoder_crudo_motor > 0 && (abs(variacion_encoder_crudo_motor) > 100) ) { // Evitar invertir pequeños rebotes
      // Si se comandó CCW, y la variación cruda fue positiva (ej. de 3500 a 100 -> variacion cruda = -3400 + 4096 = 696)
      // Es un poco confuso, mejor calcular la diferencia angular en grados con la función.
  }


  float variacion_grados_MOTOR_encoder = (float)variacion_encoder_crudo_motor * RAW_A_GRADOS;
  // Corregir el signo de la variación de grados del motor si la dirección física es CCW para que sea negativo
  if (!direccion_fisica_cw) {
      // Si la variación de grados del motor es positiva pero se comandó CCW, hacerla negativa
      // Esto asume que la lectura cruda siempre aumenta en una dirección (ej. CW)
      // y la lógica de wraparound la mantiene en un rango "pequeño".
      // Una forma más robusta es usar calcularDiferenciaAngular con los ángulos en grados.
      float ang_antes_grados = (float)encoder_antes_mov_perfil * RAW_A_GRADOS;
      float ang_despues_grados = (float)encoder_despues_mov * RAW_A_GRADOS;
      variacion_grados_MOTOR_encoder = calcularDiferenciaAngular(ang_despues_grados, ang_antes_grados);
  }


  float variacion_grados_SALIDA_calculada_del_encoder = variacion_grados_MOTOR_encoder / RELACION_REDUCTOR;

  float grados_eje_salida_objetivo_comandados = (float)(direccion_fisica_cw ? 1 : -1) * pasos_motor_a_dar_magnitud / PASOS_MOTOR_POR_GRADO_EJE_SALIDA;


  Serial1.println("\n--- Resultados del Segmento Perfil Triangular ---");
  Serial1.print("Objetivo EJE SALIDA: "); Serial1.print(grados_eje_salida_objetivo_comandados, 2); Serial1.println(" grados");
  Serial1.print("   Pasos MOTOR comandados: "); Serial1.println( (direccion_fisica_cw ? 1 : -1) * pasos_motor_a_dar_magnitud );
  Serial1.print("   V_pico_MOTOR (pps): "); Serial1.print(velocidad_pico_motor_pps, 2);
  Serial1.print(", T_total_Calculado (s): "); Serial1.println(T_total_seg_perfil, 3);
  
  Serial1.print("Encoder EJE MOTOR: Antes="); Serial1.print(encoder_antes_mov_perfil);
  Serial1.print(" / Despues="); Serial1.println(encoder_despues_mov);
  Serial1.print("   Variacion Grados EJE MOTOR (Encoder): "); Serial1.println(variacion_grados_MOTOR_encoder, 2);
  Serial1.print("   Variacion Grados EJE SALIDA (Calculada del Encoder): "); Serial1.println(variacion_grados_SALIDA_calculada_del_encoder, 2);
  
  float error_grados_salida = grados_eje_salida_objetivo_comandados - variacion_grados_SALIDA_calculada_del_encoder;

  Serial1.print("Error de posicionamiento EJE SALIDA (grados_objetivo - grados_encoder): ");
  Serial1.println(error_grados_salida, 2);
  
  float tiempo_real_s = (tiempo_fin_movimiento_micros_perfil - tiempo_inicio_movimiento_micros_perfil) / 1000000.0;
  Serial1.print("Tiempo real del movimiento (s): "); Serial1.println(tiempo_real_s, 3);
  Serial1.println("------------------------------------------------------");
}

void loop() {
  // No hacer nada en el loop principal para esta prueba, todo está en setup()
  delay(10000);
}