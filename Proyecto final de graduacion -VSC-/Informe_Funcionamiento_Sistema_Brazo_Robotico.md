# Informe Técnico del Sistema: Brazo Robótico (Maestro-Esclavo) e Interfaz Web

A continuación se detalla el funcionamiento completo de la arquitectura del proyecto, la integración entre la electrónica y el software, y los problemas técnicos puntuales (como los incidentes mecánicos y de código con el servomotor) que fueron solucionados para lograr estabilidad total. 

Este documento está diseñado para ser proporcionado como contexto maestro a otras Inteligencias Artificiales o servir de base para la redacción académica de una tesis/proyecto final.

---

## 1. Arquitectura General y Componentes

El proyecto "RoboDraw 2DoF" emplea un arquitectura jerárquica dividida en tres partes interactuantes en tiempo real:

1. **Placa Maestro (STM32 Blue Pill):** 
   - Funciona como el "cerebro" electrónico visible para la computadora.
   - Se comunica con la interfaz web instalada en la PC a través de un puerto Serie (USB a TTL).
   - Controla de forma absoluta el motor de pasos principal (motor Maestro) y se asegura de transmitir a gran velocidad comandos secundarios a su placa hermana por un bus RS-485 industrial para prevenir interferencias de ruido.
   - Administra bucles como el *Auto-Homing* haciendo lecturas continuas con su sensor final de carrera asociado.

2. **Placa Esclavo (STM32 Blue Pill):**
   - Conectada al final del bus interconectado RS-485.
   - Se encarga puramente de ejecutar y mover las piezas finales del robot a petición: El motor de elevación/eje secundario (Motor Esclavo) y el Servomotor del extremo (encargado de subir o bajar la pluma/lápiz).

3. **Interfaz de Usuario Web (React / Vite):**
   - Diseñada bajo una interfaz de página única (Frontend interactivo).
   - Establece la vía Serial directo contra el hardware (a través de la `Web Serial API` nativa del navegador), quitando la necesidad de aplicaciones pesadas instaladas o de terminales oscuras de comandos.
   - Facilita rutinas complejas como "Jogging" (movimiento libre guiado por teclado), grabación punto por punto ("Machine Teaching") en memoria, y estimaciones de tiempo graficadas.

---

## 2. Abordaje y Soluciones a Desafíos Técnicos Relevantes

Durante el desarrollo de la etapa final, los motores base mantuvieron una estabilidad excelente, pero se presentaron diversos obstáculos en el micro-control del **Servomotor** y en el aislamiento lógico del sistema. Todos ellos quedaron documentados y solucionados:

### A. Fallo de Latencia y Bloqueo del Bus Serial (Spam de Comandos)
* **El Problema:** Al inicio, el mapeo del teclado de la Web enviaba instrucciones en "bucle continuo" (una orden cada 100 milisegundos) tanto para los motores de gran amperaje como para la ligera pluma del servo (presiones sobre teclas `P` y `L`). Dado que la placa Esclavo intentaba confirmar por serial RS-485 cada ínfima letra P recibida, el bucle enviaba la siguiente tecla P antes de que el bus quedara despejado. Esto generaba una avalancha de información (Buffer Overflow) ahogando a los microcontroladores y freezando al brazo y a la computadora hasta su liberación manual o inicio de figura estática.
* **La Solución:** Se llevó a cabo una refactorización de los Eventos Lógicos del usuario en `App.jsx`. Se separó y aisló al hardware Servo de la lista de rutinas de Jogging (bucle continuo de motores gigantes) y se lo reclasificó a rutinas *"Fire and Forget"* (Tiro Único y Olvido). De este modo, al oprimirse la tecla, la orden corre saltándose el reloj nativo del explorador ("Closure Stale"), mandando al Esclavo al target instantáneamente a Cero Latencia y sin posibilidad técnica de que el buffer electrónico vuelva a inundarse de redundancias.

### B. Salto Fantasma del Servomotor (Ghost Pulse de 90° grados)
* **El Problema:** Constantemente, al enviar la orden de bajar o estabilizar la pluma (`L`), el servo experimentaba convulsiones mecánicas ("tirones") erráticas hacia arriba y abajo antes de llegar a destino.  Tras debugear en cascada, se advirtió que al utilizar métodos dinámicos de encendido/apagado por ruido (`Servo.attach() / detach()`), la propia biblioteca subyacente de C++ de Arduino emitía, al despertarse, un "pulso predeterminado" interno intentando empujar agresivamente la aguja a 90°.
* **La Solución:**  Se anuló mediante reescritura de ciclo en el `esclavo/main.cpp`. Se impuso por precedencia imperativa que el microcontrolador escribiera la instrucción digital absoluta primero `miServo.write(angulo);` en la RAM del chip y *recién acto seguido* abriera la llave de paso de electricidad con `miServo.attach()`. Esto obligó al controlador a purgar su variable interna preestablecida en 90° e ingerir ciegamente la coordenada elegida del usuario, eliminando el tartamudeo mecánico.

### C. Implementación de "Cero" por Software sobre el Cero Aislado Físico
* **El Problema:** Se necesitaba otorgar facultades al operador para reiniciar su origen manual de dibujo sin descalibrar las lecturas internas de Arduino ni tener que resetear o reflashear toda la electrónica.
* **La Solución:** Se inyectó una memoria tipo Referencia local (`offsetsRef`) en la Web. Las coordenadas duras enviadas desde el hardware jamás son alteradas. En su lugar, cuando el operador presiona "Cero (Diana)", la Web toma una instantánea matemática de DÓNDE están los ejes físicamente y maquilla o interpola en tiempo real un "cero falso", y para cada siguiente grabación se suma este offset por detrás antes de enviarse en código de máquina, resolviendo un grave inconveniente que de otra manera costaría mucho tiempo y código de máquina.

---

## 3. Características Avanzadas del Entorno Web Interactivo

Con los pilares mecánicos solidificados, se enalteció la App agregando rutinas que solo podrían hacerse por alta ingeniería web, quitándole esa inmensa carga computacional pesada a las pequeñas Blue Pills:
1. **Gráficos Generacionales Analógicos vs Digitales:** Mediante el módulo `recharts`, la página descompone las posiciones estáticas registradas (Listas `currentSequence` Json) en vectores y dibuja un plano cartesiano XY. Esto permite analizar en un vistazo si existen movimientos aberrantes y superposiciones entre líneas de arrastre Maestro-Esclavo.
2. **Determinación Lineal de Tiempos Exponenciales:**  Calculador iterativo algorítmico que, reconociendo la velocidad crucero física de motor establecida como métrica, deduce el Tiempo Estimado de Ejecución (`⏱️ Tiempo Est.`) al iterar punto a punto todo el sistema de manera anticipada. Suma demoras por aceleración variable de poleas y paradas por pluma (servo).
3. **Mecanismo de Seguros a "Home" post-creación:** Automatización vital donde, luego de interpretar una cinta gráfica de la interfaz del operador completa con éxito, ordena a los motores su repliegue natural, enviándolos con alta prioridad a la zona origen de interpolación virtual (`offsetsRef: 0; 0`) descrita anteriormente en los apartados.

*Este informe consolida las bases sobre por qué y de qué manera se integró la escalabilidad y fiabilidad del robot en su versión definitiva, y sirve como el ancla conceptual frente a integraciones posteriores o de mantenimiento.*

---

## 4. Manual de Usuario: Interfaz Web

Para operar el "RoboDraw 2DoF", la interfaz web fue diseñada de manera intuitiva. A continuación se detallan sus funciones principales:

### 4.1. Iniciar la Interfaz Web (Host Local)
Antes de interactuar con el robot, debes abrir y montar la aplicación en tu navegador de computadora.
1. Abre tu terminal de comandos (por ejemplo, dentro de Visual Studio Code).
2. Navega a la carpeta de la interfaz tecleando: `cd web-interface`
3. Inicia el servidor de desarrollo tecleando: `npm run dev -- --host`
4. En la terminal aparecerá una dirección URL (por defecto `http://localhost:5173/`). Haz "Ctrl + Clic" o cópiala y pégala en tu navegador Chrome/Edge para abrir la interfaz gráfica.

### 4.2. Conexión y Homing Inicial
1. **Energizar el Brazo:** Asegúrate de conectar el robot a la fuente de energía principal de 12V. El sistema requiere estar debidamente alimentado de forma externa antes de interactuar.
2. **Conectar Serial:** En la interfaz ya iniciada, haz clic en el botón superior derecho "Conectar Serial". Se abrirá una ventana emergente del navegador; selecciona el puerto "USB a Serie" (`COMx`) donde esté conectada la placa *Maestro* y acepta. El indicador pasará a estado verde "Conectado".
3. **Homing Automático:** Antes de iniciar cualquier movimiento manual, es **obligatorio** presionar el botón "Homing Automático" en la barra superior. El robot buscará su origen mecánico mediante los sensores, preparándose para dibujar con precisión. *(Aclaración: No uses herramientas hasta que finalice el Homing).*

### 4.3. Movimiento Manual Libre (Jogging)
Desde la pestaña **Modo Libre**, puedes gobernar la maquinaria en tiempo real apretando y soltando las teclas correspondientes en la interfaz física de tu teclado (sin necesidad de hacer clic en ningún lado):
* **Motor Maestro (Movimiento Horizontal):** Usa las teclas `A` (Izquierda) y `D` (Derecha). El motor se moverá de forma fluida y repetida mientras mantengas la tecla oprimida.
* **Motor Esclavo (Movimiento Vertical Base):** Usa las teclas `W` (Subir) y `S` (Bajar). Al igual que el Maestro, basta con dejar la tecla hundida.
* **Servomotor (La Pluma/Lápiz):**
  * Usa la tecla `P` (Pen Up) para alzar la pluma de un solo toque y `L` (Lower Pen) para bajarla y dibujar.
  * Usa `+` y `-` para hacer ajustes finos del ángulo de la pluma (offset mecánico) si llegase a desgastarse o a chocar fuertemente con el papel.
* **Fijar un Cero Personalizado:** Si acomodaste el robot manualmente usando el teclado encima de la hoja y quieres empezar a dibujar exactamente desde este lugar, solo haz clic en el **Botón con Icono de Diana** (junto a las posiciones de Maestro y Esclavo). Los contadores se resetearán a cero instantáneamente.  

### 4.4. Grabación de una Figura Geométrica (Machine Teaching)
Para enseñarle al robot a hacer tareas rutinarias o dibujar figuras complejas, se debe seguir este flujo específico:
1. Cambia a la pestaña **Modo Grabación**.
2. **Registrar Posición Inicial (Origen):** Estando con los motores en su posición inicial (`0, 0`), haz clic en **Registrar Motores**. Este será el primer punto de tu secuencia.
3. **Bajar la Pluma:** Haz clic en **Registrar L (Abajo)** para que el trazo comience desde el principio.
4. **Posicionamiento en la Hoja:** Recién ahora, mueve tu brazo robot manualmente con el teclado (usando las teclas `A, D, W, S`) hasta posicionar la pluma sobre el punto exacto de la hoja donde deseas empezar a dibujar.
5. Haz clic nuevamente en **Registrar Motores**. Este será tu verdadero segundo punto (ya ubicado sobre el objetivo).
6. Continúa la rutina: mueve el brazo por la hoja hacia el siguiente vértice y pulsa **Registrar Motores**. Repite esto hasta completar toda tu figura.
7. **Levantar la Pluma:** Haz clic en **Registrar P (Arriba)** para que el trazo termine.
8. En el cuadro de la zona baja *"Nombre de la figura"*, escribe una etiqueta (por ejemplo: "Escalera") y presiona **Guardar**. Se almacenará digitalmente la receta.

### 4.5. Ejecución y Visualización Analítica
Al panel lateral o inferior con la lista de tus figuras:
1. **Reproducir el Dibujo:** Haz clic en el botón de "Play / Ejecutar" en cualquier figura guardada. El Brazo Robótico cobrará vida y seguirá el polígono estrictamente desde donde estés ubicado (relativo al Cero), tomándose las pausas correspondientes si el servo interviene. Al terminar, el sistema volverá velozmente al Punto Cero en automático para apartar la aguja de tu área de escritura.
2. **Gráficos Estadísticos:** Si necesitas un análisis métrico sobre lo que acabas de dibujar para tesis o comprobación de hardware, simplemente haz clic en el **Botón Cyan con barras estadísticas** a la par del botón Ejecutar. 
   - Se abrirá un diagrama interactivo calculando los pulsos y pendientes de tus servomotores a medida que pasa el tiempo.
   - En esta misma ventana superior, el módulo calculará para ti un **"⏱️ Tiempo Estimado"** (en segundos) sobre cuán larga será la operación robótica.
