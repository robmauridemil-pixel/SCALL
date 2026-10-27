/*
  ===================================================================
   SCALL – Sistema de captación de agua de lluvia
   ESP32-S3-N16R8 (montaje real)
  ===================================================================
   HC-SR04 mide la distancia del sensor al agua.
     · Agua a <= DIST_ENCENDER_CM (4 cm) → enciende la motobomba (relé).
     · Agua a >= DIST_APAGAR_CM (4,5 cm) → apaga la motobomba.
       Los 0,5 cm de diferencia solo absorben el ruido del sensor.
   YF-S201 mide caudal y litros. La protección contra marcha en seco
   es opcional (PROTECCION_SIN_FLUJO, desactivada por defecto).
   El canal 2 del relé (llenado manual) queda deshabilitado: en el
   esquemático IN2 está sin conectar.

   Telemetría y control por WiFi + MQTT → página web en Vercel.
     <base>/telemetria → JSON cada 2 s y en cada cambio (retenido)
     <base>/estado     → "online" / "offline" (LWT, retenido)
     <base>/cmd        ← "AUTO" | "ON" | "OFF" | "RESET" | "RESET_LITROS"
                          | "LLENAR" | "PARAR_LLENADO"

   Funciona con broker público de prueba (sin TLS) o con un broker
   privado con TLS y usuario/clave (ver MQTT_USE_TLS más abajo).

   El control funciona aunque no haya WiFi: la red solo es para monitoreo.

   Conexiones (ver esquemático):
     HC-SR04  VCC→5V  GND→GND  TRIG→GPIO4  ECHO→1k→GPIO5 (2k de GPIO5 a GND)
     YF-S201  Rojo→5V Negro→GND Amarillo→10k→GPIO15 (20k de GPIO15 a GND)
     Relé     DC+→5V  DC-→GND  IN1→GPIO12  (IN2 sin conectar)
              COM1→+5V fuente bomba   NO1→rojo bomba   blanco bomba→- fuente
              Diodo 1N4007 en paralelo con la bomba, franja hacia el rojo
     Opcional (LLENADO_MANUAL = true):
              IN2→GPIO13  COM2→+5V fuente llenado  NO2→actuador de llenado

   Arduino IDE:
     · Placa: "ESP32S3 Dev Module" (core esp32 de Espressif 3.x)
     · USB CDC On Boot: Enabled   (para ver el Monitor Serie por el USB nativo)
     · Flash Size: 16MB  ·  PSRAM: OPI PSRAM
     · Librería: PubSubClient (Nick O'Leary) desde el Gestor de Librerías
     · Monitor Serie a 115200 baudios
  ===================================================================
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ============================ CONFIGURACIÓN ==========================
// --- WiFi (solo redes de 2,4 GHz) ---
const char* WIFI_SSID = "TU_RED_WIFI";
const char* WIFI_PASS = "TU_CLAVE_WIFI";

// --- MQTT ---
// Opción A (prueba): broker público, sin usuario ni clave.
//   MQTT_USE_TLS 0 · MQTT_HOST "broker.hivemq.com" · puerto 1883
// Opción B (recomendada): broker privado con TLS y credenciales.
//   MQTT_USE_TLS 1 · host del cluster · puerto 8883 · usuario y clave
#define MQTT_USE_TLS 0
const char*    MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;                  // TLS: 8883
const char*    MQTT_USER = "";                    // broker privado: usuario
const char*    MQTT_PASS = "";                    // broker privado: clave
#define DEVICE_ID  "men-agua-01"                  // cámbialo por uno único
#define TOPIC_BASE "potabilizadora/" DEVICE_ID
const char* T_TELE   = TOPIC_BASE "/telemetria";
const char* T_ESTADO = TOPIC_BASE "/estado";
const char* T_CMD    = TOPIC_BASE "/cmd";

// --- Pines (iguales al esquemático) ---
#define PIN_TRIG   4
#define PIN_ECHO   5
#define PIN_FLUJO  15
#define PIN_RELE   12
#define PIN_RELE_LLENADO 13

// true  = el relé se activa con HIGH (módulo de 2 canales con jumpers en HIGH)
// false = el relé se activa con LOW  (módulos "low level trigger")
const bool RELE_ACTIVO_ALTO = true;
const bool RELE_LLENADO_ACTIVO_ALTO = true;

// --- Control por nivel ---
// La bomba solo funciona mientras hay agua a 4 cm del sensor. El apagado
// queda 0,5 cm más abajo para que el ruido del HC-SR04 (±0,3 cm) no haga
// temblar el relé. El HC-SR04 no mide bien por debajo de ~2 cm.
const float    DIST_ENCENDER_CM   = 4.0;    // agua a esta distancia o menos → ON
const float    DIST_APAGAR_CM     = 4.5;    // agua a esta distancia o más   → OFF
const uint32_t T_MIN_CAMBIO_MS    = 2000;   // evita que el relé "tiemble"
const uint32_t T_SENSOR_PERDIDO_MS = 5000;  // sin lecturas válidas → apaga

// --- Llenado manual (canal 2 del relé) ---
// Por ahora queda simulado en el panel. Para habilitarlo físicamente:
// conectar IN2/COM2/NO2 y poner LLENADO_MANUAL en true.
const bool     LLENADO_MANUAL    = false;
const float    DIST_LLENO_CM     = 3.0;     // se detiene al llegar a esta distancia
const uint32_t T_MAX_LLENADO_MS  = 90000;   // corte de seguridad (90 s)

// --- Protección contra marcha en seco (caudalímetro) ---
// Desactivada por defecto: el YF-S201 mide desde ~1 L/min y una mini bomba
// de 5 V suele dar ese caudal o menos. Una falsa alarma deja la bomba
// bloqueada hasta recibir "RESET" por MQTT (sin WiFi, hasta reiniciar la
// placa). Actívala solo si el caudalímetro marca bien con la bomba andando.
const bool     PROTECCION_SIN_FLUJO = false;
const float    CAUDAL_MIN_LMIN      = 0.2;    // por debajo = "no hay flujo"
const uint32_t T_CEBADO_MS          = 10000;  // tiempo para que arranque el flujo
const float    PULSOS_POR_LITRO     = 450.0;  // YF-S201: F(Hz) = 7.5 × Q(L/min)

// --- Tiempos ---
const uint32_t T_LECTURA_US_MS   = 100;    // lectura ultrasónica
const uint32_t T_CAUDAL_MS       = 1000;   // cálculo de caudal
const uint32_t T_TELEMETRIA_MS   = 2000;   // envío a la web
const uint32_t T_REINTENTO_MS    = 5000;   // reconexión MQTT
// =====================================================================

// ----------------------------- Estado --------------------------------
enum Modo { MODO_AUTO, MODO_ON, MODO_OFF };
enum Alarma { AL_NINGUNA, AL_SIN_FLUJO, AL_SENSOR };

Modo     modo = MODO_AUTO;
Alarma   alarma = AL_NINGUNA;
bool     bomba = false;
bool     sensorOk = false;
float    distanciaCm = NAN;
float    caudalLmin = 0;
double   litrosTotal = 0;
String   motivo = "inicio";
bool     llenado = false;
String   motivoLlenado = "inactivo";

uint32_t tUltimaValida = 0, tUltimoCambio = 0, tBombaOn = 0;
uint32_t tLecturaUS = 0, tCaudal = 0, tTele = 0, tReintento = 0, tLlenado = 0;
bool     telemetriaPendiente = true;

// Caudalímetro (interrupción)
volatile uint32_t pulsos = 0;
portMUX_TYPE muxFlujo = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR isrFlujo() {
  portENTER_CRITICAL_ISR(&muxFlujo);
  pulsos++;
  portEXIT_CRITICAL_ISR(&muxFlujo);
}

// Mediana de las últimas lecturas ultrasónicas
const int N_MUESTRAS = 5;
float muestras[N_MUESTRAS];
int   nMuestras = 0, iMuestra = 0;

#if MQTT_USE_TLS
WiFiClientSecure netClient;
#else
WiFiClient netClient;
#endif
PubSubClient mqtt(netClient);

// ----------------------------- Relé ----------------------------------
void escribirRele(bool on) {
  digitalWrite(PIN_RELE, (on == RELE_ACTIVO_ALTO) ? HIGH : LOW);
}

void setBomba(bool on, const char* porque) {
  if (on == bomba) return;
  bomba = on;
  escribirRele(on);
  tUltimoCambio = millis();
  if (on) tBombaOn = millis();
  motivo = porque;
  telemetriaPendiente = true;
  Serial.printf("[BOMBA] %s (%s)\n", on ? "ENCENDIDA" : "APAGADA", porque);
}

// ------------------------- Sensor ultrasónico -------------------------
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  uint32_t us = pulseIn(PIN_ECHO, HIGH, 25000);   // máx ~4 m
  if (us == 0) return -1;                         // sin eco
  float cm = us * 0.0343f / 2.0f;
  if (cm < 1.0f || cm > 400.0f) return -1;        // fuera de rango
  return cm;
}

float mediana() {
  float t[N_MUESTRAS];
  for (int i = 0; i < nMuestras; i++) t[i] = muestras[i];
  for (int i = 1; i < nMuestras; i++) {           // ordenamiento por inserción
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
    telemetriaPendiente = true;
    Serial.printf("[NIVEL] sensor %s\n", ok ? "OK" : "SIN LECTURA");
  }
}

// ------------------------- Llenado manual ----------------------------
void escribirReleLlenado(bool on) {
  digitalWrite(PIN_RELE_LLENADO, (on == RELE_LLENADO_ACTIVO_ALTO) ? HIGH : LOW);
}

void detenerLlenado(const char* porque) {
  if (!llenado) return;
  llenado = false;
  escribirReleLlenado(false);
  motivoLlenado = porque;
  telemetriaPendiente = true;
  Serial.printf("[LLENADO] OFF (%s)\n", porque);
}

void iniciarLlenado() {
  if (!LLENADO_MANUAL || llenado) return;
  if (!sensorOk) {
    motivoLlenado = "sin lectura de nivel";
    telemetriaPendiente = true;
    return;
  }
  if (distanciaCm <= DIST_LLENO_CM) {
    motivoLlenado = "tanque lleno";
    telemetriaPendiente = true;
    return;
  }
  setBomba(false, "llenado manual");
  modo = MODO_OFF;              // no drenar el tanque al terminar
  llenado = true;
  tLlenado = millis();
  motivoLlenado = "manual";
  escribirReleLlenado(true);
  telemetriaPendiente = true;
  Serial.println("[LLENADO] ON");
}

void controlLlenado() {
  if (!llenado) return;
  if (!sensorOk) { detenerLlenado("sensor sin lectura"); return; }
  if (distanciaCm <= DIST_LLENO_CM) { detenerLlenado("tanque lleno"); return; }
  if (millis() - tLlenado >= T_MAX_LLENADO_MS) { detenerLlenado("tiempo máximo"); return; }
}

// ---------------------------- Caudalímetro ----------------------------
void actualizarCaudal() {
  uint32_t ahora = millis();
  uint32_t dt = ahora - tCaudal;
  if (dt < T_CAUDAL_MS) return;
  tCaudal = ahora;

  portENTER_CRITICAL(&muxFlujo);
  uint32_t p = pulsos;
  pulsos = 0;
  portEXIT_CRITICAL(&muxFlujo);

  float hz = p * 1000.0f / dt;
  caudalLmin = hz / 7.5f;
  litrosTotal += p / PULSOS_POR_LITRO;
}

// ------------------------------ Control -------------------------------
void controlBomba() {
  uint32_t ahora = millis();

  // El llenado manual tiene prioridad: la bomba de distribución queda inhibida.
  if (llenado) { setBomba(false, "llenado manual en curso"); return; }

  // 1) Protección: bomba encendida pero sin flujo después del cebado
  if (PROTECCION_SIN_FLUJO && bomba && alarma == AL_NINGUNA &&
      ahora - tBombaOn > T_CEBADO_MS && caudalLmin < CAUDAL_MIN_LMIN) {
    alarma = AL_SIN_FLUJO;
    setBomba(false, "alarma: sin flujo (marcha en seco)");
    return;
  }

  // 2) Con alarma de flujo activa la bomba queda apagada hasta RESET
  if (alarma == AL_SIN_FLUJO) { setBomba(false, "alarma activa"); return; }

  // 3) Alarma de sensor (solo informa; se borra sola al volver a leer)
  Alarma nueva = (!sensorOk && modo == MODO_AUTO) ? AL_SENSOR : AL_NINGUNA;
  if (nueva != alarma) { alarma = nueva; telemetriaPendiente = true; }

  // 4) Decisión según el modo
  switch (modo) {
    case MODO_ON:  setBomba(true,  "manual ON");  return;
    case MODO_OFF: setBomba(false, "manual OFF"); return;
    case MODO_AUTO: break;
  }

  if (!sensorOk) { setBomba(false, "sin lectura de nivel"); return; }
  if (ahora - tUltimoCambio < T_MIN_CAMBIO_MS) return;   // anti-rebote

  if (!bomba && distanciaCm <= DIST_ENCENDER_CM) setBomba(true,  "agua cerca del sensor");
  else if (bomba && distanciaCm >= DIST_APAGAR_CM) setBomba(false, "nivel bajó");
}

// ------------------------------- MQTT --------------------------------
const char* txtModo()   { return modo == MODO_AUTO ? "AUTO" : (modo == MODO_ON ? "ON" : "OFF"); }
const char* txtAlarma() { return alarma == AL_SIN_FLUJO ? "SIN_FLUJO" : (alarma == AL_SENSOR ? "SENSOR" : "NINGUNA"); }

void publicarTelemetria() {
  if (!mqtt.connected()) return;
  char dist[16];
  if (sensorOk) snprintf(dist, sizeof(dist), "%.1f", distanciaCm);
  else          strcpy(dist, "null");

  char json[640];
  snprintf(json, sizeof(json),
    "{\"id\":\"%s\",\"dist_cm\":%s,\"sensor_ok\":%s,\"bomba\":%s,\"modo\":\"%s\","
    "\"caudal_lmin\":%.2f,\"litros\":%.3f,\"alarma\":\"%s\",\"motivo\":\"%s\","
    "\"umbral_on_cm\":%.1f,\"umbral_off_cm\":%.1f,"
    "\"llenado\":%s,\"llenado_motivo\":\"%s\",\"llenado_max_s\":%lu,"
    "\"llenado_stop_cm\":%.1f,\"llenado_disponible\":%s,"
    "\"rssi\":%d,\"uptime_s\":%lu}",
    DEVICE_ID, dist, sensorOk ? "true" : "false", bomba ? "true" : "false", txtModo(),
    caudalLmin, litrosTotal, txtAlarma(), motivo.c_str(),
    DIST_ENCENDER_CM, DIST_APAGAR_CM,
    llenado ? "true" : "false", motivoLlenado.c_str(),
    (unsigned long)(T_MAX_LLENADO_MS / 1000), DIST_LLENO_CM,
    LLENADO_MANUAL ? "true" : "false",
    WiFi.RSSI(), (unsigned long)(millis() / 1000));

  mqtt.publish(T_TELE, json, true);
}

void alRecibir(char* topic, byte* payload, unsigned int len) {
  String cmd;
  for (unsigned int i = 0; i < len; i++) cmd += (char)payload[i];
  cmd.trim();
  cmd.toUpperCase();
  Serial.printf("[CMD] %s\n", cmd.c_str());

  if      (cmd == "AUTO") modo = MODO_AUTO;
  else if (cmd == "ON")   modo = MODO_ON;
  else if (cmd == "OFF")  modo = MODO_OFF;
  else if (cmd == "RESET") {
    if (alarma == AL_SIN_FLUJO) { alarma = AL_NINGUNA; motivo = "alarma reiniciada"; }
  }
  else if (cmd == "RESET_LITROS") litrosTotal = 0;
  else if (cmd == "LLENAR") iniciarLlenado();
  else if (cmd == "PARAR_LLENADO") detenerLlenado("detenido por el usuario");
  else return;
  tUltimoCambio = 0;            // permite responder al comando de inmediato
  telemetriaPendiente = true;
}

void mantenerMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (millis() - tReintento < T_REINTENTO_MS) return;
  tReintento = millis();

  String clientId = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.print("[MQTT] conectando... ");
  // Sin credenciales configuradas se conecta de forma anónima (broker público).
  const char* usuario = (MQTT_USER[0] != '\0') ? MQTT_USER : nullptr;
  const char* clave   = (MQTT_PASS[0] != '\0') ? MQTT_PASS : nullptr;
  // LWT: si la ESP32 se desconecta, el broker publica "offline"
  if (mqtt.connect(clientId.c_str(), usuario, clave, T_ESTADO, 1, true, "offline")) {
    Serial.println("OK");
    mqtt.publish(T_ESTADO, "online", true);
    mqtt.subscribe(T_CMD);
    telemetriaPendiente = true;
  } else {
    Serial.printf("falló (rc=%d)\n", mqtt.state());
  }
}

void mantenerWifi() {
  static bool estabaConectado = false;
  static uint32_t tIntento = 0;
  bool conectado = WiFi.status() == WL_CONNECTED;
  if (conectado && !estabaConectado)
    Serial.printf("[WiFi] conectado, IP %s\n", WiFi.localIP().toString().c_str());
  if (!conectado && millis() - tIntento > 15000) {
    tIntento = millis();
    Serial.println("[WiFi] reconectando...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  estabaConectado = conectado;
}

// ------------------------------- Setup --------------------------------
void setup() {
  // El relé arranca APAGADO antes que cualquier otra cosa
  escribirRele(false);
  pinMode(PIN_RELE, OUTPUT);
  escribirRele(false);

  escribirReleLlenado(false);
  pinMode(PIN_RELE_LLENADO, OUTPUT);
  escribirReleLlenado(false);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SCALL – Sistema de captación de agua de lluvia ===");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FLUJO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLUJO), isrFlujo, FALLING);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

#if MQTT_USE_TLS
  // Maqueta: acepta el certificado del broker sin validarlo.
  // Producción: reemplazar por netClient.setCACert(CA_PEM); con el CA del broker.
  netClient.setInsecure();
#endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(alRecibir);
  mqtt.setBufferSize(768);
  mqtt.setKeepAlive(15);

  tCaudal = millis();
}

// -------------------------------- Loop --------------------------------
void loop() {
  uint32_t ahora = millis();

  if (ahora - tLecturaUS >= T_LECTURA_US_MS) {
    tLecturaUS = ahora;
    actualizarNivel();
  }
  controlLlenado();
  actualizarCaudal();
  controlBomba();

  mantenerWifi();
  mantenerMqtt();

  if (telemetriaPendiente || millis() - tTele >= T_TELEMETRIA_MS) {
    tTele = millis();
    telemetriaPendiente = false;
    publicarTelemetria();
    Serial.printf("dist=%s cm  bomba=%s  modo=%s  caudal=%.2f L/min  litros=%.3f  alarma=%s  llenado=%s\n",
                  sensorOk ? String(distanciaCm, 1).c_str() : "--",
                  bomba ? "ON" : "OFF", txtModo(), caudalLmin, litrosTotal, txtAlarma(),
                  llenado ? "ON" : "OFF");
  }
}
