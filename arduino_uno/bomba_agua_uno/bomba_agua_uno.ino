/*
  ===================================================================
   SCALL – Sistema de captación de agua de lluvia
   Versión Arduino UNO (sin WiFi, control local)
  ===================================================================
   La bomba arranca APAGADA.
     · Algo/agua a menos de DIST_ENCENDER_CM (4 cm) del HC-SR04
       → enciende la motobomba para vaciar el tanque.
     · Nada a esa distancia (>= DIST_APAGAR_CM) → apaga la motobomba.
       Los 0,5 cm de diferencia solo absorben el ruido del sensor.
       Para vaciar más el tanque, sube DIST_APAGAR_CM (por ejemplo, a
       la distancia entre el sensor y el fondo menos 2 cm).
   El YF-S201 solo informa caudal y litros por el Monitor Serie.
   El LED integrado (pin 13) se enciende junto con la bomba.

   Conexiones (el UNO trabaja a 5 V: NO se usan los divisores
   R1–R4 del esquemático de la ESP32, los cables van directos):
     HC-SR04  VCC→5V   GND→GND   TRIG→D9   ECHO→D10
     YF-S201  Rojo→5V  Negro→GND Amarillo→D2
     Relé     DC+→5V   DC-→GND   IN1→A0    (IN2 sin conectar)
              Jumper S1 en HIGH
              COM1→+5V fuente externa   NO1→cable rojo bomba
              Cable blanco bomba→ - fuente externa
              Diodo 1N4007 en paralelo con la bomba, franja hacia el rojo
     Opcional: resistencia de 10 kΩ de IN1 a GND para que el relé no
     se active mientras el UNO arranca.

   Arduino IDE:
     · Placa: "Arduino Uno"
     · Monitor Serie a 9600 baudios
     · No necesita librerías externas
  ===================================================================
*/

// --- Pines ---
#define PIN_TRIG   9
#define PIN_ECHO   10
#define PIN_FLUJO  2      // D2 = interrupción externa INT0
#define PIN_RELE   A0     // A0 usado como salida digital
#define PIN_LED    LED_BUILTIN

// true  = el relé se activa con HIGH (jumper S1 en HIGH)
// false = el relé se activa con LOW  (módulos de 1 canal sin jumper, como el
//         SRD-05VDC-SL-C con LED rojo de power y LED verde de relé activo)
// Prueba: con bomba=OFF en el Monitor Serie, el LED verde debe estar apagado.
const bool RELE_ACTIVO_ALTO = false;

// --- Control por nivel ---
const float         DIST_ENCENDER_CM    = 4.0;   // menos de esto → bomba ON
const float         DIST_APAGAR_CM      = 4.5;   // esto o más     → bomba OFF
const unsigned long T_MIN_CAMBIO_MS     = 2000;  // evita que el relé "tiemble"
const unsigned long T_SENSOR_PERDIDO_MS = 2000;  // sin lecturas válidas → apaga

// --- Caudalímetro ---
const float PULSOS_POR_LITRO = 450.0;            // YF-S201: F(Hz) = 7.5 × Q(L/min)

// --- Tiempos ---
const unsigned long T_LECTURA_US_MS = 100;       // lectura ultrasónica
const unsigned long T_CAUDAL_MS     = 1000;      // cálculo de caudal
const unsigned long T_REPORTE_MS    = 1000;      // Monitor Serie

// ----------------------------- Estado --------------------------------
bool  bomba       = false;
bool  sensorOk    = false;
float distanciaCm = -1;
float caudalLmin  = 0;
float litrosTotal = 0;

unsigned long tUltimaValida = 0, tUltimoCambio = 0;
unsigned long tLecturaUS = 0, tCaudal = 0, tReporte = 0;

// Caudalímetro (interrupción)
volatile unsigned long pulsos = 0;
void isrFlujo() { pulsos++; }

// Mediana de las últimas lecturas ultrasónicas
const int N_MUESTRAS = 5;
float muestras[N_MUESTRAS];
int   nMuestras = 0, iMuestra = 0;

// ----------------------------- Relé ----------------------------------
void escribirRele(bool on) {
  digitalWrite(PIN_RELE, (on == RELE_ACTIVO_ALTO) ? HIGH : LOW);
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

void setBomba(bool on, const char* porque) {
  if (on == bomba) return;
  bomba = on;
  escribirRele(on);
  tUltimoCambio = millis();
  Serial.print(F("[BOMBA] "));
  Serial.print(on ? F("ENCENDIDA") : F("APAGADA"));
  Serial.print(F(" ("));
  Serial.print(porque);
  Serial.println(F(")"));
}

// ------------------------- Sensor ultrasónico -------------------------
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);   // máx ~4 m
  if (us == 0) return -1;                                // sin eco
  float cm = us * 0.0343f / 2.0f;
  if (cm < 1.0f || cm > 400.0f) return -1;               // fuera de rango
  return cm;
}

float mediana() {
  float t[N_MUESTRAS];
  for (int i = 0; i < nMuestras; i++) t[i] = muestras[i];
  for (int i = 1; i < nMuestras; i++) {                  // ordenamiento por inserción
    float v = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return t[nMuestras / 2];
}

void actualizarNivel() {
  float cm = medirDistanciaCm();
  if (cm > 0) {
    muestras[iMuestra] = cm;
    iMuestra = (iMuestra + 1) % N_MUESTRAS;
    if (nMuestras < N_MUESTRAS) nMuestras++;
    distanciaCm = mediana();
    tUltimaValida = millis();
  }
  bool ok = nMuestras > 0 && (millis() - tUltimaValida < T_SENSOR_PERDIDO_MS);
  if (ok != sensorOk) {
    sensorOk = ok;
    Serial.println(ok ? F("[NIVEL] sensor OK") : F("[NIVEL] sensor SIN LECTURA"));
  }
}

// ---------------------------- Caudalímetro ----------------------------
void actualizarCaudal() {
  unsigned long ahora = millis();
  unsigned long dt = ahora - tCaudal;
  if (dt < T_CAUDAL_MS) return;
  tCaudal = ahora;

  noInterrupts();
  unsigned long p = pulsos;
  pulsos = 0;
  interrupts();

  float hz = p * 1000.0f / dt;
  caudalLmin = hz / 7.5f;
  litrosTotal += p / PULSOS_POR_LITRO;
}

// ------------------------------ Control -------------------------------
void controlBomba() {
  if (!sensorOk) { setBomba(false, "sin lectura de nivel"); return; }
  if (millis() - tUltimoCambio < T_MIN_CAMBIO_MS) return;   // anti-rebote

  if (!bomba && distanciaCm < DIST_ENCENDER_CM)       setBomba(true,  "agua cerca del sensor");
  else if (bomba && distanciaCm >= DIST_APAGAR_CM)    setBomba(false, "nada cerca del sensor");
}

// ------------------------------- Setup --------------------------------
void setup() {
  // El relé arranca APAGADO antes que cualquier otra cosa
  pinMode(PIN_LED, OUTPUT);
  escribirRele(false);
  pinMode(PIN_RELE, OUTPUT);
  escribirRele(false);

  Serial.begin(9600);
  Serial.println(F("\n=== SCALL – Arduino UNO ==="));
  Serial.println(F("Bomba APAGADA. Enciende con agua a menos de 4 cm."));

  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FLUJO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLUJO), isrFlujo, FALLING);

  tCaudal = millis();
}

// -------------------------------- Loop --------------------------------
void loop() {
  unsigned long ahora = millis();

  if (ahora - tLecturaUS >= T_LECTURA_US_MS) {
    tLecturaUS = ahora;
    actualizarNivel();
  }
  actualizarCaudal();
  controlBomba();

  if (millis() - tReporte >= T_REPORTE_MS) {
    tReporte = millis();
    Serial.print(F("dist="));
    if (sensorOk) Serial.print(distanciaCm, 1); else Serial.print(F("--"));
    Serial.print(F(" cm  bomba="));
    Serial.print(bomba ? F("ON") : F("OFF"));
    Serial.print(F("  caudal="));
    Serial.print(caudalLmin, 2);
    Serial.print(F(" L/min  litros="));
    Serial.println(litrosTotal, 3);
  }
}
