# SCALL — Sistema de captación de agua de lluvia

Panel de monitoreo y control para el tanque de agua de lluvia de SCALL, con ESP32-S3.
El firmware mide el nivel con un HC-SR04, acciona la bomba de distribución por el
canal 1 del relé, mide el caudal con un YF-S201 y deja preparado el llenado manual
por el canal 2 (por ahora simulado en el panel). El panel web muestra la telemetría
en tiempo real por MQTT y permite comandar el sistema desde el navegador.

El control de la bomba funciona aunque no haya WiFi: la red solo se usa para
monitorear y comandar a distancia.

---

## 0. Versión Arduino UNO (sin WiFi)

Sketch: [`arduino_uno/bomba_agua_uno/bomba_agua_uno.ino`](arduino_uno/bomba_agua_uno/bomba_agua_uno.ino).
Diagrama: [`arduino_uno/esquematico_arduino_uno.png`](arduino_uno/esquematico_arduino_uno.png).
Hace el mismo control por nivel que la ESP32, pero sin panel web ni MQTT:

- La bomba arranca **apagada**.
- Si hay algo o agua a **menos de 4 cm** del HC-SR04, la bomba enciende para vaciar el tanque.
- Si no hay nada a esa distancia (≥ 4,5 cm), la bomba se apaga. Los 0,5 cm solo
  absorben el ruido del sensor. Para vaciar más, subir `DIST_APAGAR_CM`.
- El LED integrado (pin 13) se enciende junto con la bomba.
- El YF-S201 solo informa caudal y litros en el Monitor Serie (9600 baudios).

El UNO trabaja a 5 V, igual que los sensores, así que **se eliminan los divisores
R1–R4** del esquemático de la ESP32: los cables van directos. El relé se mantiene,
porque un pin del UNO entrega ~20 mA y la bomba necesita cientos.

| Componente | Cable | Arduino UNO |
| --- | --- | --- |
| HC-SR04 | VCC | 5V |
| HC-SR04 | GND | GND |
| HC-SR04 | TRIG | D9 |
| HC-SR04 | ECHO | D10 (directo, sin resistencias) |
| YF-S201 | Rojo | 5V |
| YF-S201 | Negro | GND |
| YF-S201 | Amarillo | D2 (directo, sin resistencias) |
| Relé 2 canales | DC+ | 5V |
| Relé 2 canales | DC- | GND |
| Relé 2 canales | IN1 | A0, usado como salida digital (jumper S1 en HIGH) |
| Relé 2 canales | IN2 | sin conectar |

El circuito de potencia no cambia: COM1 → +5 V de la fuente externa, NO1 → cable
rojo de la bomba, cable blanco de la bomba → − de la fuente externa, y el diodo
1N4007 en paralelo con la bomba con la franja hacia el cable rojo. El GND de la
fuente externa no se une al UNO.

Opcional: una resistencia de 10 kΩ de IN1 a GND evita que el relé se active
durante el segundo que tarda el UNO en arrancar.

Para cargarlo: Arduino IDE → **Archivo → Abrir** el `.ino` → **Herramientas → Placa →
Arduino Uno** → elegir el puerto COM → **Subir**. No necesita librerías externas.

---

## 1. Arquitectura

```
  HC-SR04 ─────────┐
  YF-S201 ─────────┼── ESP32-S3 ── WiFi 2.4 GHz ── Broker MQTT ── WSS ── Navegador
  Relé 1 + bomba ──┤        │                          │
  Relé 2 + llenado ┘        └── control local           └── telemetría / estado / cmd
```

- La ESP32 publica telemetría cada 2 segundos y en cada cambio relevante.
- El broker retiene el último mensaje, así el panel muestra el estado apenas abre.
- La ESP32 publica `online` al conectar y el broker publica `offline` por LWT si se cae.
- El panel publica comandos en el tópico `/cmd`.

---

## 2. Hardware

| Componente | Conexión |
| --- | --- |
| HC-SR04 VCC | 5 V de la placa |
| HC-SR04 GND | GND común |
| HC-SR04 TRIG | GPIO 4 |
| HC-SR04 ECHO | GPIO 5, con divisor 1 kΩ en serie y 2 kΩ a GND |
| YF-S201 rojo | 5 V de la placa |
| YF-S201 negro | GND común |
| YF-S201 amarillo | GPIO 15, con divisor 10 kΩ en serie y 20 kΩ a GND |
| Relé VCC / GND | 5 V / GND de la placa |
| Relé IN1 | GPIO 12 |
| Relé IN2 | GPIO 13 |
| Relé COM1 | +5 V de la fuente de la bomba de distribución |
| Relé NO1 | cable rojo de la bomba de distribución |
| Bomba cable blanco | negativo de la fuente de la bomba |
| Diodo 1N4007 | en paralelo con la bomba, cátodo hacia el cable rojo |
| Relé COM2 | +5 V de la fuente de llenado |
| Relé NO2 | positivo de la bomba o electroválvula de llenado |
| Diodo de llenado | 1N4007 en paralelo con la bomba de llenado |

Notas de montaje:

- Los GPIO del ESP32-S3 trabajan a 3,3 V. Los divisores de tensión de ECHO y del
  caudalímetro son obligatorios: sin ellos se daña el pin.
- Cada bomba va a su propia fuente de 5 V a través de su canal del relé. No alimentar
  las bombas desde el pin 5V de la placa.
- Unir las tierras de la placa, los sensores, el relé y todas las fuentes.
- El módulo de relé debe tener los jumpers en HIGH y `RELE_ACTIVO_ALTO = true`.
  El canal 2 usa `RELE_LLENADO_ACTIVO_ALTO = true`.
- No usar GPIO 35, 36 ni 37: están reservados para la PSRAM del módulo N16R8.
- Este montaje es para bombas de 5 V de maqueta. No conmutar 220 V con este relé.
- El actuador de llenado es opcional. Con `LLENADO_MANUAL = false` el panel
  deshabilita el botón y el llenado solo se prueba en modo demostración.

El esquemático completo está en `esquematico_bomba_esp32s3_rele2canales.png`.
El esquemático original deja `IN2` sin conectar; el llenado manual agrega esa
conexión: `IN2` → GPIO 13 y el canal 2 del relé para el actuador de llenado.

---

## 3. Estructura del proyecto

```
SCALL/
├── bomba_agua_esp32s3_2/
│   └── bomba_agua_esp32s3_2.ino          Firmware de la ESP32-S3
├── arduino_uno/
│   ├── bomba_agua_uno/bomba_agua_uno.ino Firmware para Arduino UNO (sin WiFi)
│   ├── esquematico_arduino_uno.png       Diagrama de conexiones del UNO
│   └── esquematico_arduino_uno.svg       Diagrama editable
├── index.html                            Panel web (un solo archivo, sin build)
├── assets/
│   ├── scall-mark.png                    Emblema para el encabezado y el favicon
│   └── scall-logo.jpeg                   Logo completo para el pie
├── info/
│   ├── logo.jpeg                         Logo original entregado
│   └── Paleta de colores.jpeg            Paleta original entregada
├── esquematico_bomba_esp32s3_rele2canales.png
└── README.md
```

La identidad visual usa la paleta entregada en `info/`: cian `#04C4D9`, celeste
`#B6ECF2`, turquesa `#04D9D9`, verde `#93BF50` y marrón `#8C5B49`. El panel conserva
un rojo funcional para las alarmas críticas.

---

## 4. Inicio rápido sin hardware

1. Abrir `index.html` en Chrome, Edge o Firefox.
2. Ir a **Ajustes** (arriba a la derecha).
3. Activar **Modo demostración** y guardar.
4. El panel genera datos locales con el mismo ciclo de histéresis del firmware:
   llena el tanque, enciende la bomba a 4 cm, la apaga a 4,5 cm y acumula litros.
5. Arrastrar la barra **Nivel** al lado del tanque para mover el nivel con el mouse.
   Al llevarlo a 4 cm, la bomba se enciende automáticamente y el caudalímetro
   simulado muestra `OK`; al bajarlo a 4,5 cm, la bomba se detiene.

Al arrastrar la barra se desactiva la lluvia automática de la demo para que el
usuario tenga el control del nivel. En modo real la barra es de solo lectura y
refleja la medición del HC-SR04.

El modo demostración no se conecta al broker. Sirve para revisar la interfaz,
probar los controles y tener un respaldo visual si falla la red durante una
presentación. Se desactiva desde el mismo cuadro de ajustes o desde el aviso
superior.

---

## 5. Firmware

### 5.1 Arduino IDE

1. Instalar Arduino IDE 2.x.
2. En **Boards Manager**, instalar el core **esp32 by Espressif** (3.x).
3. En **Library Manager**, instalar **PubSubClient** de Nick O'Leary.
   `WiFiClientSecure` ya viene incluido en el core.
4. Configurar la placa:

   | Opción | Valor |
   | --- | --- |
   | Board | ESP32S3 Dev Module |
   | USB CDC On Boot | Enabled |
   | Flash Size | 16MB |
   | PSRAM | OPI PSRAM |
   | Upload Speed | 921600 o la que funcione mejor |
   | Monitor Serie | 115200 baudios |

5. Abrir `bomba_agua_esp32s3_2/bomba_agua_esp32s3_2.ino` y completar:

   ```cpp
   const char* WIFI_SSID = "TU_RED_WIFI";   // solo 2,4 GHz
   const char* WIFI_PASS = "TU_CLAVE_WIFI";
   ```

6. Cargar el sketch y abrir el Monitor Serie. Debe aparecer la IP asignada y,
   si el broker está bien configurado, `[MQTT] conectando... OK`.

### 5.2 Umbrales de nivel

```cpp
const float DIST_ENCENDER_CM = 4.0;   // agua a 4 cm o menos del sensor → bomba ON
const float DIST_APAGAR_CM   = 4.5;   // agua a 4,5 cm o más del sensor → bomba OFF
```

La bomba solo funciona mientras hay agua a 4 cm del sensor. El HC-SR04 no mide
bien por debajo de 2 cm, así que 4 cm deja 2 cm de margen sobre la zona muerta.
El apagado queda 0,5 cm más abajo solo para absorber el ruido del sensor
(±0,3 cm después de la mediana); sin esa diferencia el relé temblaría justo en
el umbral.

Con una diferencia tan corta la bomba arranca y se detiene con poco volumen de
agua. Si el relé cicla de más, subir `DIST_APAGAR_CM` o `T_MIN_CAMBIO_MS`.

### 5.3 Protecciones

- **Marcha en seco (opcional, desactivada):** con `PROTECCION_SIN_FLUJO = true`,
  si la bomba lleva 10 s encendida y el caudal es menor a 0,2 L/min, se apaga y
  queda bloqueada hasta enviar `RESET`. Viene desactivada porque el YF-S201 mide
  desde ~1 L/min y una mini bomba de 5 V puede quedar por debajo: una falsa
  alarma dejaría la bomba bloqueada (sin WiFi, hasta reiniciar la placa).
- **Encendido automático:** en AUTO, al llegar a 4 cm la bomba de distribución
  arranca y el caudalímetro se monitorea mientras está encendida. El panel muestra
  el estado `OK`, `Cebando` o `Sin flujo`.
- **Sensor de nivel:** si no hay lecturas válidas durante 5 s, la bomba se apaga
  en modo AUTO.
- **Anti-rebote:** cambios de estado separados por al menos 2 s.
- **Mediana de 5 muestras:** reduce lecturas falsas del ultrasónico.
- **Llenado manual:** cuando está habilitado, se detiene al llegar a 3 cm, a los
  90 s o si se pierde el sensor. Mientras dura, la bomba de distribución queda
  inhibida y el modo pasa a OFF.

### 5.4 Llenado manual

Por el momento el llenado está **simulado en el panel**: `LLENADO_MANUAL = false`.
El firmware no activa el canal 2 y el botón queda deshabilitado en modo real con el
aviso correspondiente. El modo demostración permite probar todo el flujo sin
hardware.

Para habilitarlo físicamente hace falta una segunda bomba o una electroválvula.

Cableado:

- `IN2` → GPIO 13.
- `COM2` → +5 V de la fuente de llenado.
- `NO2` → positivo de la bomba o electroválvula.
- Negativo del actuador → negativo de la fuente de llenado.
- Diodo 1N4007 en paralelo con la bomba, cátodo hacia el positivo.
- Masa de la fuente de llenado unida a la masa común.

Con el llenado habilitado o en modo demostración, **Llenar tanque**:

1. Pide confirmación.
2. Activa el canal 2 del relé.
3. Pasa el modo de distribución a `OFF` y deja la bomba de distribución inhibida.
4. Se detiene automáticamente al llegar a 3 cm (`DIST_LLENO_CM`), a los 90 s
   (`T_MAX_LLENADO_MS`) o si se pierde la lectura del sensor.
5. Se puede detener en cualquier momento con **Detener llenado**.

El corte por tiempo es una protección básica: no hay sensor de flujo en la línea de
llenado. Ajustar `T_MAX_LLENADO_MS` al volumen real del tanque y verificar que el
actuador no quede funcionando en seco. Con `LLENADO_MANUAL = false` (valor por
defecto), el panel deshabilita el botón en modo real y permite probarlo en
demostración. Para habilitarlo, conectar el canal 2 y poner la constante en `true`.

---

## 6. Broker MQTT

El firmware y el panel soportan dos configuraciones.

### 6.1 Opción A — Broker público (solo pruebas)

Es la configuración por defecto. No requiere cuenta ni credenciales.

```cpp
#define MQTT_USE_TLS 0
const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASS = "";
```

En el panel:

```
Broker:  wss://broker.hivemq.com:8884/mqtt
Usuario: (vacío)
Clave:   (vacío)
```

Advertencia: cualquiera que conozca el tópico puede leer los datos y publicar
comandos. No usar este broker para una instalación real ni con datos sensibles.

### 6.2 Opción B — Broker privado con credenciales (recomendado)

#### B1. HiveMQ Cloud

HiveMQ Cloud ofrece un plan **Starter** con 15 días de prueba sin tarjeta.
El antiguo plan **Serverless gratuito** se retira el 31/12/2026 y ya no permite
crear clusters nuevos; si ya existe uno, funciona hasta esa fecha.

Pasos:

1. Entrar a [hivemq.com](https://www.hivemq.com) y elegir **Start Free**.
2. Crear un cluster **Cloud Starter** (elegir la región más cercana).
3. En **Overview**, copiar:
   - la URL del cluster (por ejemplo `xxxxx.s1.eu.hivemq.cloud`);
   - el puerto TLS para MQTT (habitualmente 8883);
   - el puerto WebSocket seguro (8884).
4. En **Access Management → Credentials**, crear un usuario y una clave.
   Asignar el rol **Allow All** para una prueba rápida, o un rol con permisos
   limitados a los tópicos del proyecto.
5. Configurar el firmware:

   ```cpp
   #define MQTT_USE_TLS 1
   const char* MQTT_HOST = "xxxxx.s1.eu.hivemq.cloud";
   const uint16_t MQTT_PORT = 8883;
   const char* MQTT_USER = "usuario_del_cluster";
   const char* MQTT_PASS = "clave_del_cluster";
   ```

6. Configurar el panel:

   ```
   Broker:  wss://xxxxx.s1.eu.hivemq.cloud:8884/mqtt
   Usuario: usuario_del_cluster
   Clave:   clave_del_cluster
   ```

El sketch usa `netClient.setInsecure()` para aceptar el certificado del broker
sin validarlo. Es aceptable para una maqueta, pero en producción conviene fijar
el CA del broker con `netClient.setCACert(CA_PEM);` y pegar el certificado
correspondiente.

#### B2. Mosquitto local (gratis, misma red WiFi)

Sirve para una demostración privada sin depender de Internet. Requiere una PC o
Raspberry Pi encendida durante la demo.

1. Instalar Mosquitto.
2. Crear el archivo de claves:

   ```bash
   mosquitto_passwd -c mosquitto.passwd usuario
   ```

3. Configurar `mosquitto.conf`:

   ```conf
   allow_anonymous false
   password_file /ruta/a/mosquitto.passwd

   listener 1883
   protocol mqtt

   listener 9001
   protocol websockets
   ```

4. Reiniciar el servicio y verificar que la PC tenga IP fija o reserva DHCP.
5. Firmware:

   ```cpp
   #define MQTT_USE_TLS 0
   const char* MQTT_HOST = "192.168.0.50";   // IP de la PC
   const uint16_t MQTT_PORT = 1883;
   const char* MQTT_USER = "usuario";
   const char* MQTT_PASS = "clave";
   ```

6. Panel, si se abre desde la misma red mediante HTTP:

   ```
   Broker:  ws://192.168.0.50:9001
   Usuario: usuario
   Clave:   clave
   ```

   Abrir el panel con un servidor local (sección 7.1) para que el origen sea
   `http://` y el navegador permita la conexión `ws://`.

Importante: si el panel se publica en Vercel (HTTPS), el navegador bloquea
`ws://` por contenido mixto. En ese caso se necesita un broker con `wss://`
(HiveMQ Cloud u otro servicio administrado) o configurar TLS en Mosquitto.

#### B3. Otro broker administrado

Cualquier broker con TLS y WebSocket seguro funciona. Solo hay que completar en
el panel la URL `wss://...`, el usuario y la clave, y en el firmware el host, el
puerto TLS, `MQTT_USE_TLS 1` y las mismas credenciales.

---

## 7. Panel web

### 7.1 Abrir localmente

La forma más simple es abrir `index.html` con doble clic. También se puede servir
con un servidor local:

```bash
py -m http.server 8000
```

o, si está instalado Node.js:

```bash
npx serve .
```

Luego abrir `http://localhost:8000`.

### 7.2 Configuración del panel

En **Ajustes**:

| Campo | Qué poner |
| --- | --- |
| Broker MQTT | `wss://...` para HTTPS, `ws://...` para HTTP local |
| Tópico base | Debe coincidir con `TOPIC_BASE` del sketch |
| Usuario / Clave | Credenciales del broker privado; vacíos en broker público |
| Altura útil del tanque | Distancia real desde la cara del sensor al fondo, en cm |
| Modo demostración | Datos simulados locales, sin broker |

La configuración se guarda en `localStorage` del navegador. Las credenciales
nunca se incluyen en el HTML ni se envían a otro servidor que no sea el broker.

### 7.3 Calibración del nivel

El sensor mide la distancia que hay entre su cara y la superficie del agua. Para
convertir esa distancia en porcentaje, el panel necesita la altura útil del
tanque:

```
nivel (%) = (altura útil − distancia medida) / altura útil × 100
```

Medir con una regla desde la cara del sensor hasta el fondo del tanque y cargar
ese valor en **Ajustes → Altura útil del tanque**. El valor por defecto es 30 cm.

Los umbrales de encendido y apagado no se configuran en el panel: llegan en la
telemetría desde el firmware (`umbral_on_cm` y `umbral_off_cm`).

### 7.4 Desplegar en Vercel

El panel es un archivo estático: no tiene build, ni variables de entorno, ni
backend.

#### Opción con Vercel CLI

Requiere Node.js 18 o superior.

```bash
npm install -g vercel
vercel login
vercel
vercel --prod
```

Responder:

- **Set up and deploy**: sí.
- **Which scope**: la cuenta propia.
- **Link to existing project**: no.
- **Project name**: `scall` o el nombre que se prefiera.
- **Directory**: `.` (la carpeta del proyecto).
- **Build command**: vacío.
- **Output directory**: `.` o vacío.
- **Development command**: vacío.

La primera ejecución crea un despliegue de prueba; `vercel --prod` publica la
versión definitiva. Vercel entrega HTTPS automáticamente, por eso el broker debe
ser `wss://`.

#### Opción con GitHub

1. Subir el proyecto a un repositorio.
2. En [vercel.com/new](https://vercel.com/new), importar el repositorio.
3. Framework preset: **Other**.
4. Build command: vacío. Output directory: `.`.
5. Deploy.

Cada push a la rama principal vuelve a desplegar.

#### Antes de publicar

- No incluir credenciales en `index.html`.
- Tener en cuenta que la página publicada es accesible para cualquiera que tenga
  la URL. Sin las credenciales del broker no puede conectarse, pero si el panel
  debe ser privado, usar la protección de despliegues de Vercel o un dominio con
  acceso restringido.
- Probar el panel publicado con el broker real antes de la presentación.

### 7.5 Qué muestra el panel

- **Tanque:** nivel porcentual, distancia cruda, umbrales, zona muerta y barra
  lateral de nivel. En demostración la barra se arrastra con el mouse; en modo real
  es de solo lectura y sigue al sensor.
- **Bomba:** estado, motivo del último cambio, modo, tiempo en marcha, ciclos desde
  que se abrió el panel y estado del caudalímetro (`OK`, `Cebando`, `Sin flujo`).
- **Caudal:** litros por minuto, total acumulado y diferencia desde la apertura.
- **Alarmas:** marcha en seco y sensor sin lectura.
- **Historial:** últimos 10 minutos de caudal, nivel o litros acumulados.
- **Control:** AUTO, encendido y apagado forzados, reinicio de alarma y del
  totalizador.
- **Llenado manual:** estado, último motivo, botón de inicio/parada y aviso de
  seguridad del canal 2.
- **Eventos:** conexión, cambios de bomba, alarmas, llenado y comandos enviados.
- **Dispositivo:** identificador, broker, tópico, RSSI, uptime y cadencia.

Estados de conexión:

| Estado | Significado |
| --- | --- |
| Conectando al broker | Abriendo el WebSocket |
| Esperando telemetría | Broker conectado, sin datos del dispositivo |
| En línea | Telemetría reciente (menos de 8 s) |
| Telemetría retrasada | El dispositivo dejó de publicar |
| Dispositivo fuera de línea | El broker recibió el LWT `offline` |
| Simulación activa | Modo demostración local |

---

## 8. Contrato MQTT

Tópicos (con `DEVICE_ID = "men-agua-01"`):

El identificador MQTT es independiente del nombre comercial SCALL. Se mantiene
`men-agua-01` para no romper un dispositivo ya flasheado; si se cambia, hay que
actualizar el tópico base en **Ajustes**.

| Tópico | Dirección | Retenido | Contenido |
| --- | --- | --- | --- |
| `potabilizadora/men-agua-01/telemetria` | ESP32 → panel | Sí | JSON de estado |
| `potabilizadora/men-agua-01/estado` | ESP32 → panel | Sí | `online` / `offline` |
| `potabilizadora/men-agua-01/cmd` | Panel → ESP32 | No | Comando de texto |

Ejemplo de telemetría:

```json
{
  "id": "men-agua-01",
  "dist_cm": 3.8,
  "sensor_ok": true,
  "bomba": true,
  "modo": "AUTO",
  "caudal_lmin": 1.42,
  "litros": 7.25,
  "alarma": "NINGUNA",
  "motivo": "agua cerca del sensor",
  "umbral_on_cm": 4.0,
  "umbral_off_cm": 4.5,
  "llenado": false,
  "llenado_motivo": "inactivo",
  "llenado_max_s": 90,
  "llenado_stop_cm": 3.0,
  "llenado_disponible": false,
  "rssi": -61,
  "uptime_s": 125
}
```

Comandos:

| Comando | Efecto |
| --- | --- |
| `AUTO` | Control por nivel con histéresis |
| `ON` | Bomba de distribución forzada encendida |
| `OFF` | Bomba de distribución forzada apagada |
| `RESET` | Borra la alarma de marcha en seco |
| `RESET_LITROS` | Pone el totalizador en cero |
| `LLENAR` | Inicia el llenado manual por el canal 2 |
| `PARAR_LLENADO` | Detiene el llenado manual |

`LLENAR` y `PARAR_LLENADO` requieren `LLENADO_MANUAL = true` en el firmware. En modo
demostración funcionan siempre, porque el llenado es simulado en el navegador.

Si se cambia `DEVICE_ID` en el firmware, hay que actualizar el tópico base en
**Ajustes** del panel.

Con el firmware por defecto, `llenado_disponible` llega en `false` y el panel
deshabilita el botón de llenado en modo real. Pasa a `true` al habilitar el canal 2
con `LLENADO_MANUAL = true`.

---

## 9. Solución de problemas

| Síntoma | Revisar |
| --- | --- |
| El panel dice “Sin conexión al broker” | URL y puerto; `wss://` si la página es HTTPS; usuario y clave; que la red no bloquee el puerto 8884 |
| “Dispositivo fuera de línea” | Alimentación de la ESP32; que esté en la misma red; Monitor Serie; LWT del broker |
| “Telemetría retrasada” | La ESP32 publica cada 2 s; revisar WiFi, RSSI y conexión MQTT |
| “Marcha en seco” | Solo con `PROTECCION_SIN_FLUJO = true`. Bomba cebada, manguera sin aire, cableado del YF-S201, `CAUDAL_MIN_LMIN`; enviar `RESET` |
| El nivel no coincide con el agua | Cargar la altura útil real del tanque; el sensor mide distancia, no volumen |
| El HC-SR04 da lecturas locas | Mantener 4 cm de umbral, revisar 5 V y divisores, evitar ecos y superficies blandas |
| El relé no conmuta | Jumpers en HIGH, `RELE_ACTIVO_ALTO`, IN1 en GPIO 12, masa común |
| El llenado no arranca en modo real | Es lo esperado con `LLENADO_MANUAL = false`; probar en modo demostración o habilitar el canal 2 |
| La barra de nivel no se mueve | En modo real es de solo lectura; activar **Modo demostración** para arrastrarla |
| El llenado no sube el nivel | Actuador sin conectar, fuente de llenado apagada, IN2/COM2/NO2 mal cableados |
| El llenado no se detiene | Revisar `DIST_LLENO_CM` y el sensor; el corte de seguridad es `T_MAX_LLENADO_MS`; usar **Detener llenado** |
| El llenado activa el relé pero no hay agua | El canal 2 no tiene bomba/electroválvula o la fuente de llenado está cerrada |
| Vercel bloquea la conexión | Una página HTTPS solo puede abrir `wss://`; usar un broker con TLS |
| HiveMQ rechaza la conexión | Credenciales correctas, cluster activo, puerto TLS correcto, permisos del rol |
| La página publicada muestra una versión vieja | Volver a desplegar y recargar sin caché |

---

## 10. Lista de verificación para la presentación

- WiFi de 2,4 GHz y credenciales cargadas en el sketch.
- Monitor Serie mostrando `WiFi conectado` y `[MQTT] conectando... OK`.
- Panel abierto con el broker, tópico y credenciales correctas.
- Altura útil del tanque calibrada.
- Prueba en AUTO: arrastrar la barra de nivel a 4 cm, verificar bomba ON y
  caudalímetro `OK`; bajar a 4,5 cm y verificar bomba OFF.
- Prueba real: acercar una tapa plana a 4 cm del HC-SR04 y verificar bomba ON;
  retirarla y verificar bomba OFF.
- Prueba de comandos ON, OFF y AUTO desde el panel.
- Prueba de llenado manual en modo demostración: arranque, corte a 3 cm y parada
  con **Detener llenado**.
- Confirmar que la bomba de distribución queda inhibida durante el llenado simulado.
- El llenado físico queda deshabilitado hasta conectar el canal 2.
- Si se activa `PROTECCION_SIN_FLUJO`: prueba de alarma de marcha en seco y
  reinicio con `RESET`.
- Modo demostración probado como respaldo si falla la red.
- Fuente de la bomba conectada. Su GND no va a la ESP32: el relé aísla la bomba.
- Panel publicado en Vercel probado desde otro dispositivo.

---

## 11. Próximos pasos

- Broker privado con permisos por dispositivo y tópicos restringidos.
- Historial persistente en InfluxDB, Supabase o similar (hoy el gráfico vive en
  memoria y cubre 10 minutos).
- Notificaciones por Telegram o correo ante alarmas.
- Actualizaciones OTA del firmware.
- Soporte para varios tanques o dispositivos.

---

## 12. Créditos

Proyecto educativo SCALL de captación de agua de lluvia. Usa:

- [PubSubClient](https://github.com/knolleary/pubsubclient) para MQTT en Arduino.
- [MQTT.js](https://github.com/mqttjs/MQTT.js) para MQTT sobre WebSocket en el navegador.
- Tipografías Inter y JetBrains Mono desde Google Fonts.
- Logo y paleta de marca entregados en `info/`.
