/*
  ARDMX EVO — firmware ESP32
  Equivalent funcional de l'ARDMX4 (Arduino Mega): controlador DMX512 amb 4
  escenes i transicions, cicle sincronitzat amb música MP3 (DFPlayer Mini),
  controlat per BLE (GATT) amb el mateix protocol de trames `!Vxx=valor$` /
  `!Vxx=?$` que ja fan servir l'ARDMX4 i l'ARDMX One.

  Migrat de Bluetooth Classic (SPP) a BLE (2026-08) — rèplica exacta del
  mateix canvi ja fet i validat en maquinari a ardmx-one-firmware (Classic no
  és accessible des d'apps de tercers a iOS sense certificació MFi, BLE sí).
  El Mega (ardmx4-firmware) es manté amb Classic/HC-06, sense tocar.

  Disseny GATT — deliberadament els MATEIXOS UUIDs que ardmx-one-firmware,
  no uns de nous: és el mateix transport per al mateix protocol `!Vxx=
  valor$`, i el producte ja es distingeix pel handshake V64/nom un cop
  connectat (vegeu DeviceIdentificationService a l'app), no pel transport en
  si — reutilitzar-los permet que l'app trobi tots dos productes en el
  mateix escaneig BLE sense cap canvi a l'app.
    - Servei: 74fdf89b-a063-48f4-837d-03462d2b3687
    - Característica d'escriptura (Write + Write Without Response):
      c7e05764-94cb-4a2f-8cd4-4751163c58ad
    - Característica de notificació: dd2a9ece-4964-4f42-b986-36719d38b2a3

  NimBLE-Arduino (no la llibreria BLE del core, basada en Bluedroid): molt
  més lleugera en flash/RAM i més fiable en notificacions llargues — mateix
  motiu que a l'ARDMX One, encara més rellevant aquí perquè aquest firmware
  ja anava justet de flash (87% amb BluetoothSerial) abans de migrar.

  Fragmentació per MTU i concurrència: idèntic a ardmx-one-firmware —
  sendFrame() fragmenta les respostes llargues (V69, V71) en trossos de mida
  MTU; els bytes rebuts per BLE (tasca pròpia de l'stack, no loop()) es
  passen per una cua FreeRTOS (bleRxQueue) perquè processFrame()/
  handleWrite() i tota la resta d'estat global (V[], canalsData...) seguint
  tocant-se només des de loop(), mai des del callback onWrite() — encara més
  important aquí que a l'ARDMX One, ja que l'estat de cicle/escenes d'aquest
  firmware és molt més gran i una condició de carrera hi seria molt pitjor.

  La lògica de cicle/escenes/transicions (NouCicle, CanviEstat,
  AvancarCicleSiCal, cridaTransicio, GravacioIntervals...) i la gestió del
  DFPlayer (Reproductor, GestioDFPlayer) estan portades gairebé literalment
  del Mega (ARDMX4.ino V4.18) — és la font de la veritat funcional, no s'ha
  reinventat res. El que canvia és tot el que depenia del maquinari del
  Mega: persistència (EEPROM amb adreces fixes -> NVS en trossos petits),
  protocol (llibreria VirtuinoCM -> parser propi, el mateix que ja fa
  servir l'ARDMX One), DMX (llibreria Conceptinetics -> esp_dmx), i el LED
  (interrupció de Timer1 -> sondeig a loop()).

  Mapa de l'índex V[] (0-50 idèntic al Mega, per compatibilitat total amb
  les pantalles de l'app ja existents; 62+ són índexs nous o amb significat
  diferent, decidits explícitament perquè no col·lisionin de forma
  perillosa amb els altres dos firmwares — vegeu la memòria del projecte):
    V00      número de cançó MP3 (0 = sense música)
    V01-V03  valor actual (0-255) dels 3 canals "visibles"
    V04-V06  número de canal DMX assignat a cada un d'ells
    V07      avança/retrocedeix de grup de 3 canals (-1/0/+1)
    V09      escena activa (1-4)
    V10      estat actual del cicle (1-8, 0 = aturat)
    V11      selector principal (1=Automàtic,2=Trigger,3-6=Escena fixa,7=Config)
    V12/V13  Play/Pausa (pantalla de Cicle)
    V14/V15  temps transcorregut / temps total del cicle (s)
    V16      volum DFPlayer (0-30)
    V18      nombre d'escenes actives (1-4)
    V21-V28  temps acumulats dels 8 períodes del cicle (s)
    V31-V33  mode de transició (0=gradual,1=inicial,2=final) dels 3 canals visibles
    V35      ordre de canvi d'escena activa (delta)
    V39      MAX_CANALS (510, constant)
    V40      nombre de canals actius (1-510)
    V41/V42  arma/confirma el reset de fàbrica
    V50      pantalla activa (mateix AppScreen que l'ARDMX4)
    V62      versió de firmware (text, només lectura)
    V63      nom Bluetooth (text; escriure'l reinicia l'ESP32 — com l'One)
    V64      identificació del dispositiu (JSON, només lectura — com l'One)
    V65-V67  nom (text) del canal DMX assignat a cada slot 1-3
    V68      nom del pessebre (text)
    V69      descripció (text)
    V71      consulta/assignació massiva d'UN canal explícit, les 4 escenes
             de cop (valor+mode) més el nom — no toca la selecció dels
             sliders. Nou índex (ni V63 del Mega ni V70 de l'One: cap dels
             dos formats existents cobria "4 escenes + nom" alhora):
               V71=N                              -> consulta
               V71=N|v1|m1|v2|m2|v3|m3|v4|m4|nom  -> assigna
             Resposta sempre "v1|m1|v2|m2|v3|m3|v4|m4|nom".

  Maquinari:
    - MAX485 (direcció automàtica) per DMX: GPIO22=TX, GPIO21=RX (no usat),
      UART DMX_NUM_1 (mai DMX_NUM_2, vegeu la nota a dmxInit()).
    - DFPlayer Mini per UART1: GPIO17 (TX ESP32, amb resistència sèrie
      ~1kΩ) -> RX DFPlayer; TX DFPlayer -> divisor de tensió (1kΩ+2kΩ) ->
      GPIO16 (RX ESP32). Alimentat a 5V.
    - LED d'estat a GPIO2 (resistència 220-330Ω en sèrie).
    - Trigger extern (mode manual, EstatSelector==2) a GPIO4, INPUT_PULLUP.
*/

#include <Arduino.h>
#include <esp_dmx.h>

// La pila per defecte de loop() (8 KB) provoca crashes intermitents
// (Guru Meditation Error) després d'escriure a la NVS — confirmat en
// maquinari real amb l'ARDMX One. Cal ampliar-la abans que es cridi
// qualsevol funció que hi escrigui.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <Preferences.h>
#include <DFRobotDFPlayerMini.h>

// ---------------------------------------------------------------------------
// Configuració de maquinari
// ---------------------------------------------------------------------------

constexpr dmx_port_t DMX_PORT = DMX_NUM_1;  // mai DMX_NUM_2 — crash confirmat en maquinari real (ARDMX One)
constexpr int DMX_TX_PIN = 22;
constexpr int DMX_RX_PIN = 21;   // no s'usa activament, DMX només surt
constexpr int DMX_RTS_PIN = -1;  // -1 = MAX485 amb commutació automàtica, no cal gestionar cap pin

constexpr int STATUS_LED_PIN = 2;
constexpr int TRIGGER_PIN = 4;  // mode Trigger (EstatSelector==2), INPUT_PULLUP

// Freqüència d'enviament DMX limitada (com l'ARDMX One): enviar sense
// límit maximitza la finestra en què una trama activa pot coincidir amb
// una escriptura NVS i provocar un crash del controlador DMX.
constexpr uint32_t DMX_SEND_INTERVAL_MS = 25;

// Buffer intern reservat per a 510 canals (límit físic), encara que el
// valor per defecte/inicial de canals actius sigui 100 — així es pot
// pujar numeroCanals en calent des de l'app sense recompilar ni migrar
// dades. CHANNEL_BUFFER_SIZE (múltiple de CHANNEL_CHUNK_SIZE) és una mica
// més gran que MAX_CANALS perquè els trossos NVS tinguin totes la mateixa
// mida (mateix patró que l'ARDMX One, on MAX_DMX_CHANNEL=512 ja hi requeia just).
constexpr int MAX_CANALS = 510;
constexpr int CHANNEL_CHUNK_SIZE = 32;
constexpr int CHANNEL_BUFFER_SIZE = 512;  // 16 * 32, cobreix MAX_CANALS amb marge
constexpr int CHANNEL_CHUNK_COUNT = CHANNEL_BUFFER_SIZE / CHANNEL_CHUNK_SIZE;  // 16
constexpr int DEFAULT_NUMERO_CANALS = 100;  // valor inicial/de fàbrica (com l'ARDMX4 actual)

constexpr uint32_t SAVE_DEBOUNCE_MS = 500;

const char *DEFAULT_BLUETOOTH_NAME = "ARDMXEVO";
constexpr int MAX_BLUETOOTH_NAME_LENGTH = 15;

constexpr int MAX_CHANNEL_NAME_LENGTH = 15;
constexpr int MAX_PESSEBE_NAME_LENGTH = 96;
constexpr int MAX_DESCRIPTION_LENGTH = 384;

const char *FIRMWARE_VERSION_TEXT = "ARDMX EVO v2.0";

// PIN de connexió — opcional (String buida = desactivat, comportament de
// sempre). Quan n'hi ha un, cap V/T es contesta ni s'aplica (V64, V73, V75
// són les úniques excepcions) fins que l'app l'enviï correcte per V73. Es
// desa en clar a NVS (com el nom Bluetooth): no protegeix contra algú amb
// accés físic a la placa, només contra connectar-se sense voler al dispositiu
// del veí en una trobada de pessebristes — no cal xifrar-lo per a això.
constexpr int PIN_LENGTH = 4;
String storedPin = "";
bool pinAuthenticated = false;

// El JSON d'identificació (V64) ara inclou si cal PIN, així que ja no és
// una constant fixa — es construeix a cada petició.
String buildIdentifyJson() {
  // L'app no llegeix aquest camp per l'EVO (només ho fa per distingir One v1
  // de v2) — es bumpa igualment per simetria/debug amb l'altra placa.
  String json = "{\"tipus\":\"ARDMX_EVO\",\"firmware\":\"2.0.0\",";
  json += "\"num_canals_max\":510,\"pin\":";
  json += (storedPin.length() > 0) ? "true" : "false";
  json += "}";
  return json;
}

// ---------------------------------------------------------------------------
// Configuració BLE — mateixos UUIDs que ardmx-one-firmware, deliberadament
// (vegeu el comentari de capçalera).
// ---------------------------------------------------------------------------

constexpr const char *BLE_SERVICE_UUID = "74fdf89b-a063-48f4-837d-03462d2b3687";
constexpr const char *BLE_WRITE_CHAR_UUID = "c7e05764-94cb-4a2f-8cd4-4751163c58ad";
constexpr const char *BLE_NOTIFY_CHAR_UUID = "dd2a9ece-4964-4f42-b986-36719d38b2a3";

constexpr uint16_t BLE_PREFERRED_MTU = 247;
constexpr size_t BLE_RX_CHUNK_MAX = 256;

struct BleRxChunk {
  uint8_t data[BLE_RX_CHUNK_MAX];
  size_t length;
};

// ---------------------------------------------------------------------------
// Estat global
// ---------------------------------------------------------------------------

Preferences prefs;

NimBLEServer *bleServer = nullptr;
NimBLECharacteristic *bleNotifyCharacteristic = nullptr;
QueueHandle_t bleRxQueue = nullptr;
volatile bool bleClientConnected = false;
volatile uint16_t bleConnHandle = BLE_HS_CONN_HANDLE_NONE;
// UART2, no UART1: el DMX (esp_dmx) reclama DMX_NUM_1 (el mateix
// perifèric UART1) — provar d'inicialitzar-hi també el DFPlayer amb
// HardwareSerial(1) petava (Guru Meditation Error / LoadProhibited dins
// uartBegin) perquè els dos intentaven controlar el mateix UART. UART2
// (pins per defecte GPIO16/17, els mateixos ja validats) queda lliure.
HardwareSerial dfSerial(2);

// Mateix patró que el Mega: els índexs 0-59 del protocol es guarden
// directament en aquest array, i tota la lògica de cicle/escenes
// (portada gairebé literalment) hi llegeix/escriu tal com feia amb V[] al
// Mega. Els índexs >=60 (versió, nom BT, handshake, noms de canal,
// pessebre, descripció, protocol massiu) són sempre de text i es
// gestionen a part (vegeu processFrame()).
constexpr int V_SIZE = 60;
float V[V_SIZE];

// ---- Transicions globals (substitueixen el mode per canal 0/1/2) ----
// Cada transició s'aplica a TOTS els canals alhora durant el pas d'escena
// i+1 -> ((i+1)%4)+1. SALT és l'únic tipus que fa servir saltPercent
// (percentatge del temps de la transició en què es produeix el salt
// instantani); la resta l'ignoren.
enum TipusTransicio : uint8_t { LINEAL = 0, SALT = 1, EASE_IN = 2, EASE_OUT = 3 };

struct Transicio {
  TipusTransicio tipus;
  uint8_t saltPercent;  // 0-100, només rellevant si tipus==SALT
};

// Interpola el valor (0-255) d'un canal entre l'escena origen (v0) i destí
// (v1) segons el progrés de la transició (t_pct, 0-1000 mil·lèsimes). Tot en
// enters (sense floats) per eficiència a l'ESP32.
uint8_t interpolar(uint8_t v0, uint8_t v1, uint16_t t_pct, TipusTransicio tipus, uint8_t salt_pct) {
  switch (tipus) {
    case SALT:
      return (t_pct < (uint16_t)salt_pct * 10) ? v0 : v1;
    case EASE_OUT: {
      // Ràpid al principi, s'alenteix al final: f(t) = 1-(1-t)^2
      int32_t inv = 1000 - (int32_t)t_pct;
      int32_t f = 1000 - (inv * inv) / 1000;
      return (uint8_t)(v0 + ((int32_t)(v1 - v0) * f) / 1000);
    }
    case EASE_IN: {
      // Lent al principi, accelera al final: f(t) = t^2
      int32_t f = ((int32_t)t_pct * t_pct) / 1000;
      return (uint8_t)(v0 + ((int32_t)(v1 - v0) * f) / 1000);
    }
    case LINEAL:
    default:
      return (uint8_t)(v0 + ((int32_t)(v1 - v0) * t_pct) / 1000);
  }
}

// ---- Paràmetres generals (equivalent de ParametresGenerals del Mega) ----
struct ParametresGenerals {
  int EscenaActiva;
  int NumeroEscenes;
  int NumeroMusica;
  int NivellVolum;
  int EstatSelector;
  uint32_t tempsPeriodes[8];  // microsegons
  int NumeroCanals;
};
ParametresGenerals Parametres;
bool paramsDirty = false;

// ---- Dades de canal: 4 valors (0-255) I les 4 transicions (tipus+%salt)
// PRÒPIES de cada canal — ja NO hi ha un mode per canal (0/1/2) ni
// transicions globals compartides: cada canal DMX pot fer, p.ex., un SALT
// sobtat mentre un altre fa un LINEAL suau durant la mateixa transició
// d'escena.
struct CanalData {
  uint8_t valors[4];
  Transicio transicions[4];  // transicions[i]: escena i+1 -> escena ((i+1)%4)+1
};
CanalData canalsData[CHANNEL_BUFFER_SIZE];
float valorActual[CHANNEL_BUFFER_SIZE];  // valor DMX que s'està enviant ara mateix (per la interpolació)
bool canalsDirty = false;
// Quins dels 16 trossos NVS tenen canvis pendents — editar UN canal (p.ex.
// arrossegar un slider o canviar-li la transició) només toca 1-2 trossos,
// pero saveCanals() reescrivia sempre els 16 sencers a cada desat
// debounced. Cada escriptura NVS bloqueja loop()/DMX uns ms; amb
// SAVE_DEBOUNCE_MS tan curt (500ms) i 16 trossos sencers a cada desat, un
// simple ús normal (moure sliders, canviar transicions) es notava com un
// petit "encallament" periòdic. Ara saveCanals()/saveNames() només
// reescriuen els trossos marcats aquí (mateix mecanisme a ardmx-one-firmware).
bool canalsChunkDirty[CHANNEL_CHUNK_COUNT] = {false};

char channelNames[CHANNEL_BUFFER_SIZE][MAX_CHANNEL_NAME_LENGTH + 1];
bool namesDirty = false;
bool namesChunkDirty[CHANNEL_CHUNK_COUNT] = {false};

String pessebeName;
String descripcio;

uint32_t lastChangeMillis = 0;

// ---- Selecció de canals visibles (V01-03/V04-06) ----
int Canal_1 = 1;
int Canal_2 = 2;
int Canal_3 = 3;
int numeroCanals = DEFAULT_NUMERO_CANALS;  // V40 — sempre <= MAX_CANALS

// ---- Estat d'escenes/cicle ----
int EscenaActiva = 1;      // V09
int NumeroEscenes = 4;     // V18
int EstatSelector = 0;     // V11
int MusicaMP3 = 0;         // V00
int NivellVolum = 20;      // V16
String EstatPlay = "Stop";
bool cicloEnCurso = false;
bool triggerAnterior = false;

int num_periodes = NumeroEscenes * 2;
uint32_t Temps[8];             // durada de cada període, microsegons
uint32_t TempsAcumulat[8];     // temps acumulat (per validar seqüència), segons
uint32_t tiempoTotalCiclo = 0; // microsegons

uint32_t tempsActualEstat = 0, referenciaTempsEstat = 0;
uint32_t tempsActualCicle = 0, referenciaTempsCicle = 0;
uint32_t tempsActualTransicio = 0, referenciaTempsTransicio = 0;
uint32_t contadorPuntTransicio = 0, numeroPuntsTransicio = 0;
constexpr uint32_t tempsCiclesTransicio = 10000;  // microsegons, >= 10000
int EstatActual = 0, EstatAntic = 0;

uint32_t pausaCicleInici = 0;  // per descomptar la pausa dels rellotges del cicle en reprendre

String text1 = " ";  // estat/error, mostrat a l'app (V61)

// ---- Reset de fàbrica ----
bool resetArmed = false;

// ---- LED d'estat ----
enum ModeLed {
  LED_NORMAL,
  LED_CICLE,
  LED_ARRANCADA,
  LED_INICIALITZANT_USB,
  LED_SENSE_USB,
};
ModeLed modeLed = LED_ARRANCADA;
bool inicialitzantUSB = false;
bool dfplayerUSBDisponible = false;

// ---- Bluetooth ----
String btFrameBuffer;
String btDeviceName;

// Declaracions avançades (definides més avall, calen per l'ordre de crida)
void markCanalsDirty(int channelIndex0);
void markNamesDirty(int channelIndex0 = -1);
void markParamsDirty();
void replyText(int index, const char *text);

// ---------------------------------------------------------------------------
// Reproductor — gestió del DFPlayer Mini (portat gairebé literalment del
// Mega V4.17/V4.18: connexió per events, no reinicialització repetida,
// reintents en segon pla).
// ---------------------------------------------------------------------------
class Reproductor {
 private:
  DFRobotDFPlayerMini *player;
  bool inicializado;
  int volumen;
  int pista;
  const int numCanciones = 4;

 public:
  Reproductor(HardwareSerial &serialPort) : inicializado(false), volumen(20), pista(1) {
    player = new DFRobotDFPlayerMini();
    intentarInicializar(serialPort);
  }

  bool intentarInicializar(HardwareSerial &serialPort) {
    inicializado = false;

    Serial.println(F("Intentant inicialitzar DFPlayer Mini..."));

    if (!player->begin(serialPort)) {
      inicializado = false;
      dfplayerUSBDisponible = false;
      Serial.println(F("DFPlayer no disponible. Sortida d'audio en repòs."));
      return false;
    }

    inicializado = true;
    player->volume(volumen);
    dfplayerUSBDisponible = true;

    // Marge perquè el DFPlayer s'acabi d'assentar just després de begin()
    // — confirmat en maquinari real que, sense això, la primera ordre
    // play() immediatament posterior es pot perdre en silenci.
    delay(1500);

    Serial.println(F("DFPlayer Mini inicialitzat correctament."));
    return true;
  }

  // Consulta (sense bloquejar) events del DFPlayer (inserció/extracció
  // d'USB...). Es crida a cada volta del loop() — no es torna a
  // inicialitzar mai el DFPlayer per saber si hi ha USB, es dedueix
  // sempre dels events.
  void comprobarEventos() {
    if (!inicializado) return;

    while (player->available()) {
      const uint8_t tipus = player->readType();
      switch (tipus) {
        case DFPlayerUSBOnline:
        case DFPlayerUSBInserted:
        case DFPlayerCardUSBOnline:
          dfplayerUSBDisponible = true;
          Serial.println(F("DFPlayer: USB detectat."));
          break;
        case DFPlayerUSBRemoved:
          dfplayerUSBDisponible = false;
          player->stop();
          Serial.println(F("DFPlayer: USB extret. Reproducció aturada."));
          break;
        default:
          break;
      }
    }
  }

  void reproducir(int nuevaPista) {
    if (nuevaPista <= 0 || !inicializado || nuevaPista > numCanciones) {
      Serial.println(F("Reproductor no inicialitzat o pista fora de rang."));
      return;
    }
    pista = nuevaPista;
    player->play(pista);
  }

  void ajustarVolumen(int nuevoVolumen) {
    if (nuevoVolumen < 0 || nuevoVolumen > 30) return;
    volumen = nuevoVolumen;
    if (!inicializado) return;
    player->volume(volumen);
  }

  void pausar() {
    if (!inicializado) return;
    player->pause();
  }

  void reanudar() {
    if (!inicializado) return;
    player->start();
  }

  void detener() {
    if (!inicializado) return;
    player->stop();
  }

  bool estaInicializado() { return inicializado; }
};

Reproductor *miReproductor;

// ---------------------------------------------------------------------------
// Canals — funcions equivalents a la classe CanalDMX del Mega, adaptades a
// un model de dades pla (array de structs) en lloc d'objectes amb
// persistència EEPROM incrustada.
// ---------------------------------------------------------------------------

// Aplica directament el valor d'una escena fixa al canal i el transmet.
// Rep un "estat" de cicle (0-7: parell=escena fixa, senar=transició), no un
// índex d'escena directe — tots els punts de crida (NouCicle, CanviEstat,
// aplicarSelector des d'Escenes()/EstatSelector) passen EstatActual o un
// valor derivat d'ell sense dividir per 2, igual que feia originalment
// `actualizarCanalsFix()` al Mega (`valores[EstatActual/2]`). Sense aquesta
// divisió, l'escena 2 llegia els valors de l'escena 3 i les escenes 3/4
// llegien fora dels límits de `valors[4]` (mida 4) — confirmat en maquinari
// real: canviar d'escena barrejava valors d'altres escenes.
void actualizarCanalFix(int i, int estat) {
  const int escenaIndex = estat / 2;
  valorActual[i] = canalsData[i].valors[escenaIndex];
}

// Un pas d'interpolació entre l'escena origen i la següent, segons el tipus
// de transició PRÒPIA d'aquest canal (canalsData[i].transicions[escenaIndex])
// — substitueix l'antic mode per canal (0/1/2), però ara cada canal té el
// seu propi tipus/percentatge, no un de compartit per tots. t_pct (0-1000)
// és el progrés ja calculat per cridaTransicio(), compartit per tots els
// canals d'aquest tick (només el tipus de corba varia per canal).
void actualizarCanalTransicio(int i, int estatActual, uint16_t t_pct) {
  const int escenaIndex = estatActual / 2;  // mateixa divisió que actualizarCanalFix (bug-fix #1)
  const uint8_t v0 = canalsData[i].valors[escenaIndex];
  // % NumeroEscenes (no % 4 fix): amb menys de 4 escenes actives, la
  // darrera transició torna a l'escena 1, no a la següent posició fixa
  // de l'array (que podria ser una escena inactiva/no configurada).
  const uint8_t v1 = canalsData[i].valors[(escenaIndex + 1) % NumeroEscenes];
  const Transicio &tr = canalsData[i].transicions[escenaIndex];
  valorActual[i] = interpolar(v0, v1, t_pct, tr.tipus, tr.saltPercent);
}

void enviarCanalEscena(int i, int escenaIndex) {
  if (escenaIndex < 0 || escenaIndex >= 4) return;
  valorActual[i] = canalsData[i].valors[escenaIndex];
}

// Assigna un valor a una escena concreta d'un canal, des de l'edició en
// directe dels 3 sliders visibles (V01-03) — desa immediatament (com feia
// el Mega), a diferència de la resta de canals d'una assignació massiva
// (V71) que no desen fins que se'ls demana explícitament.
void guardarEnviarValor(int i, int escenaIndex, int nuevoValor) {
  if (escenaIndex < 0 || escenaIndex >= 4) return;
  canalsData[i].valors[escenaIndex] = constrain(nuevoValor, 0, 255);
  valorActual[i] = canalsData[i].valors[escenaIndex];
  markCanalsDirty(i);
}

// ---------------------------------------------------------------------------
// DMX
// ---------------------------------------------------------------------------

uint8_t dmxData[DMX_PACKET_SIZE];  // índex 0 = start code (sempre 0), 1..N = canals

void dmxInit() {
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {};
  dmx_driver_install(DMX_PORT, &config, personalities, 0);
  dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_RTS_PIN);
}

// Còpia valorActual[] (calculat per actualizarCanalFix/Transicio) cap al
// buffer DMX i l'envia. Es crida com a màxim cada DMX_SEND_INTERVAL_MS.
void dmxSendFrame() {
  for (int i = 0; i < numeroCanals; i++) {
    dmxData[i + 1] = (uint8_t)constrain((int)valorActual[i], 0, 255);
  }
  const int packetSize = numeroCanals + 1;
  dmx_write(DMX_PORT, dmxData, packetSize);
  // No dmx_wait_sent() here on purpose: dmx_send_num() already waits
  // internally (up to 23ms) for the PREVIOUS frame to finish sending before
  // starting a new one, so the transmission itself never overlaps. Our own
  // explicit wait used to additionally block until the CURRENT frame
  // finished transmitting (up to ~23ms at 510 channels) — with transitions
  // ticking every 10ms, that was enough to make a running cycle visibly
  // stutter/jump above ~100 channels. Confirmed on hardware up to the full
  // 510-channel universe with no output corruption after removing it.
  dmx_send_num(DMX_PORT, packetSize);
}

void apagarTotsElsCanals() {
  for (int i = 0; i < CHANNEL_BUFFER_SIZE; i++) {
    valorActual[i] = 0;
  }
  Serial.println(F("Tots els canals DMX apagats."));
}

// ---------------------------------------------------------------------------
// Persistència (NVS)
// ---------------------------------------------------------------------------

void loadParams() {
  prefs.begin("ardmxevo", false);
  if (prefs.isKey("params")) {
    const size_t bytesRead = prefs.getBytes("params", &Parametres, sizeof(Parametres));
    if (bytesRead != sizeof(Parametres)) {
      Serial.println(F("ERROR llegint paràmetres — es descarten."));
      memset(&Parametres, 0, sizeof(Parametres));
    }
  } else {
    memset(&Parametres, 0, sizeof(Parametres));
  }
  prefs.end();

  if (Parametres.NumeroEscenes < 1 || Parametres.NumeroEscenes > 4) {
    // Primer arrencada o dada invàlida: valors de fàbrica.
    Parametres.EscenaActiva = 1;
    Parametres.NumeroEscenes = 4;
    Parametres.NumeroMusica = 0;
    Parametres.NivellVolum = 20;
    Parametres.EstatSelector = 7;
    for (int i = 0; i < 8; i++) Parametres.tempsPeriodes[i] = 5000000UL;
    Parametres.NumeroCanals = DEFAULT_NUMERO_CANALS;
  }
  Parametres.NumeroCanals = constrain(Parametres.NumeroCanals, 1, MAX_CANALS);
}

void saveParams() {
  prefs.begin("ardmxevo", false);
  if (prefs.putBytes("params", &Parametres, sizeof(Parametres)) != sizeof(Parametres)) {
    Serial.println(F("ERROR desant paràmetres a NVS"));
  }
  prefs.end();
  paramsDirty = false;
}

void markParamsDirty() {
  paramsDirty = true;
  lastChangeMillis = millis();
}

// Trossos de 32 canals (256 bytes/tros: 32 × 8 bytes de CanalData) — mai
// un blob únic (nvs_set_blob té un límit pràctic de pocs KB per entrada,
// confirmat que falla en silenci si es supera, vegeu ARDMX One).
void loadCanals() {
  prefs.begin("ardmxevo", false);
  const size_t chunkBytes = CHANNEL_CHUNK_SIZE * sizeof(CanalData);
  Serial.printf("[NVS] loadCanals: chunkBytes=%u sizeof(CanalData)=%u\n", (unsigned)chunkBytes,
                (unsigned)sizeof(CanalData));
  // Validació per tros independent, NO tot-o-res: abans, un sol tros amb
  // la mida antiga (p.ex. desat abans que CanalData guanyés
  // transicions[4]) feia caure TOT canalsData a 0, esborrant també els
  // trossos correctes — i com que saveCanals() ara només reescriu els
  // trossos tocats (canalsChunkDirty[]), un tros mai editat es podia
  // quedar amb la mida antiga indefinidament. Confirmat en maquinari real
  // (ardmx-one-firmware, mateix bug): valors/noms/transicions ja editats
  // van desaparèixer sencers en arrencar de nou.
  for (int chunk = 0; chunk < CHANNEL_CHUNK_COUNT; chunk++) {
    const String key = "chv" + String(chunk);
    void *dest = &canalsData[chunk * CHANNEL_CHUNK_SIZE];
    bool chunkOk = prefs.isKey(key.c_str());
    if (chunkOk) {
      const size_t bytesRead = prefs.getBytes(key.c_str(), dest, chunkBytes);
      chunkOk = (bytesRead == chunkBytes);
      if (!chunkOk) {
        Serial.printf("[NVS] loadCanals: clau %s ha llegit %u bytes (esperava %u), es reinicia nomes aquest tros\n",
                      key.c_str(), (unsigned)bytesRead, (unsigned)chunkBytes);
      }
    } else {
      Serial.printf("[NVS] loadCanals: clau %s no existeix, es reinicia nomes aquest tros\n", key.c_str());
    }
    if (!chunkOk) {
      memset(dest, 0, chunkBytes);
    }
  }
  prefs.end();

  for (int i = 0; i < CHANNEL_BUFFER_SIZE; i++) {
    valorActual[i] = canalsData[i].valors[0];
  }
}

void saveCanals() {
  Serial.println(F("[NVS] saveCanals: començant..."));
  prefs.begin("ardmxevo", false);
  const size_t chunkBytes = CHANNEL_CHUNK_SIZE * sizeof(CanalData);
  for (int chunk = 0; chunk < CHANNEL_CHUNK_COUNT; chunk++) {
    if (!canalsChunkDirty[chunk]) continue;
    const String key = "chv" + String(chunk);
    const void *src = &canalsData[chunk * CHANNEL_CHUNK_SIZE];
    const size_t written = prefs.putBytes(key.c_str(), src, chunkBytes);
    if (written != chunkBytes) {
      Serial.printf("[NVS] ERROR desant dades de canal (tros %d): escrits %u de %u bytes\n", chunk,
                    (unsigned)written, (unsigned)chunkBytes);
    }
    canalsChunkDirty[chunk] = false;
  }
  prefs.end();
  canalsDirty = false;
  Serial.println(F("[NVS] saveCanals: fet"));
}

// channelIndex0 (0-based): marca només el tros que conté aquest canal com a
// pendent (vegeu canalsChunkDirty[]) — un simple canvi d'1 canal ja no força
// reescriure els 16 trossos sencers.
void markCanalsDirty(int channelIndex0) {
  Serial.println(F("[NVS] markCanalsDirty"));
  canalsChunkDirty[channelIndex0 / CHANNEL_CHUNK_SIZE] = true;
  canalsDirty = true;
  lastChangeMillis = millis();
}

// Només per a un reset de fàbrica (memset complet, cal reescriure-ho tot).
void markAllCanalsDirty() {
  for (int i = 0; i < CHANNEL_CHUNK_COUNT; i++) canalsChunkDirty[i] = true;
  canalsDirty = true;
}

void loadNames() {
  prefs.begin("ardmxevo", false);
  // Validació per tros independent — mateix motiu que loadCanals().
  const size_t chunkBytes = CHANNEL_CHUNK_SIZE * (MAX_CHANNEL_NAME_LENGTH + 1);
  for (int chunk = 0; chunk < CHANNEL_CHUNK_COUNT; chunk++) {
    const String key = "chn" + String(chunk);
    char *dest = &channelNames[chunk * CHANNEL_CHUNK_SIZE][0];
    bool chunkOk = prefs.isKey(key.c_str());
    if (chunkOk) {
      const size_t bytesRead = prefs.getBytes(key.c_str(), dest, chunkBytes);
      chunkOk = (bytesRead == chunkBytes);
    }
    if (!chunkOk) {
      memset(dest, 0, chunkBytes);
    }
  }

  pessebeName = prefs.isKey("pessebe") ? prefs.getString("pessebe", "") : "";
  descripcio = prefs.isKey("descripcio") ? prefs.getString("descripcio", "") : "";

  prefs.end();
}

void saveNames() {
  prefs.begin("ardmxevo", false);
  const size_t chunkBytes = CHANNEL_CHUNK_SIZE * (MAX_CHANNEL_NAME_LENGTH + 1);
  for (int chunk = 0; chunk < CHANNEL_CHUNK_COUNT; chunk++) {
    if (!namesChunkDirty[chunk]) continue;
    const String key = "chn" + String(chunk);
    const char *src = &channelNames[chunk * CHANNEL_CHUNK_SIZE][0];
    if (prefs.putBytes(key.c_str(), src, chunkBytes) != chunkBytes) {
      Serial.printf("ERROR desant noms de canal (tros %d)\n", chunk);
    }
    namesChunkDirty[chunk] = false;
  }
  if (prefs.putString("pessebe", pessebeName) == 0 && pessebeName.length() > 0) {
    Serial.println(F("ERROR desant el nom del pessebre"));
  }
  if (prefs.putString("descripcio", descripcio) == 0 && descripcio.length() > 0) {
    Serial.println(F("ERROR desant la descripció"));
  }
  prefs.end();
  namesDirty = false;
}

// channelIndex0 (0-based, -1 = pessebeName/descripcio, que no viuen en cap
// tros i saveNames() sempre reescriu igualment): mateix mecanisme que
// markCanalsDirty().
void markNamesDirty(int channelIndex0) {
  if (channelIndex0 >= 0) namesChunkDirty[channelIndex0 / CHANNEL_CHUNK_SIZE] = true;
  namesDirty = true;
  lastChangeMillis = millis();
}

void markAllNamesDirty() {
  for (int i = 0; i < CHANNEL_CHUNK_COUNT; i++) namesChunkDirty[i] = true;
  namesDirty = true;
}

void loadBtName() {
  prefs.begin("ardmxevo", false);
  btDeviceName = prefs.isKey("btname") ? prefs.getString("btname", DEFAULT_BLUETOOTH_NAME)
                                        : String(DEFAULT_BLUETOOTH_NAME);
  prefs.end();
}

void loadPin() {
  prefs.begin("ardmxevo", false);
  storedPin = prefs.getString("pin", "");
  prefs.end();
}

// ---------------------------------------------------------------------------
// Sanejament de text
// ---------------------------------------------------------------------------

String sanitizeName(const String &rawInput) {
  String clean = "";
  for (unsigned int i = 0; i < rawInput.length(); i++) {
    const char c = rawInput.charAt(i);
    if (isAlphaNumeric(c) || c == '_') clean += c;
    if ((int)clean.length() >= MAX_BLUETOOTH_NAME_LENGTH) break;
  }
  return clean;
}

// Només xifres, exactament PIN_LENGTH — qualsevol altra cosa (buit, massa
// curt, no numèric) es considera invàlida (String buida de retorn).
String sanitizePin(const String &rawInput) {
  String clean = "";
  for (unsigned int i = 0; i < rawInput.length(); i++) {
    const char c = rawInput.charAt(i);
    if (isDigit(c)) clean += c;
  }
  if (clean.length() != PIN_LENGTH) return "";
  return clean;
}

// Talla a maxBytes BYTES, sense partir mai un caràcter UTF-8 pel mig
// (bytes de continuació amb el patró de bits 10xxxxxx, màscara 0xC0==0x80).
String sanitizeText(const String &rawInput, int maxBytes) {
  String clean = "";
  for (unsigned int i = 0; i < rawInput.length(); i++) {
    const char c = rawInput.charAt(i);
    if (c != '!' && c != '$' && c != '|') clean += c;
  }
  if ((int)clean.length() <= maxBytes) return clean;

  int cut = maxBytes;
  while (cut > 0 && (clean.charAt(cut) & 0xC0) == 0x80) cut--;
  return clean.substring(0, cut);
}

// ---------------------------------------------------------------------------
// LED d'estat — mateixos 5 modes que el Mega, sondejats a loop() en lloc
// d'una interrupció de Timer1 (no cal a l'ESP32).
// ---------------------------------------------------------------------------

void actualitzarModeLed() {
  if (inicialitzantUSB) {
    modeLed = LED_INICIALITZANT_USB;
  } else if (MusicaMP3 > 0 && !dfplayerUSBDisponible) {
    modeLed = LED_SENSE_USB;
  } else if (EstatSelector == 1 || cicloEnCurso) {
    modeLed = LED_CICLE;
  } else {
    modeLed = LED_NORMAL;
  }
}

void updateStatusLed() {
  static uint32_t lastTick = 0;
  static int contador = 0;

  const uint32_t now = millis();
  if (now - lastTick < 200) return;  // mateix "tick" de 200ms que el Timer1 del Mega
  lastTick = now;
  contador++;

  switch (modeLed) {
    case LED_NORMAL:
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case LED_CICLE:
      digitalWrite(STATUS_LED_PIN, (contador % 5) == 4 ? LOW : HIGH);
      break;
    case LED_ARRANCADA:
    case LED_INICIALITZANT_USB:
      digitalWrite(STATUS_LED_PIN, (contador % 2) == 0 ? HIGH : LOW);
      break;
    case LED_SENSE_USB:
      switch (contador % 10) {
        case 0:
        case 2:
          digitalWrite(STATUS_LED_PIN, HIGH);
          break;
        default:
          digitalWrite(STATUS_LED_PIN, LOW);
          break;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Cicle / escenes — portat gairebé literalment del Mega (ARDMX4.ino V4.18)
// ---------------------------------------------------------------------------

bool PrepararReproductorSiCal() {
  if (MusicaMP3 <= 0) {
    dfplayerUSBDisponible = true;
    return false;
  }
  return miReproductor->estaInicializado() && dfplayerUSBDisponible;
}

void GestioDFPlayer() {
  if (MusicaMP3 <= 0) return;

  if (miReproductor->estaInicializado()) {
    miReproductor->comprobarEventos();
    return;
  }

  const uint32_t REINTENT_DFPLAYER_MS = 5000;
  static uint32_t ultimReintent = 0;
  if (millis() - ultimReintent < REINTENT_DFPLAYER_MS) return;
  ultimReintent = millis();

  inicialitzantUSB = true;
  actualitzarModeLed();

  miReproductor->intentarInicializar(dfSerial);

  inicialitzantUSB = false;
  dfplayerUSBDisponible = miReproductor->estaInicializado();
  actualitzarModeLed();
}

bool verificarSequenciaTemps(int m) {
  if (V[21] <= 0) return false;
  for (int x = 1; x < NumeroEscenes * 2; x++) {
    if (V[x + 21] <= V[x + 20]) return false;
  }
  return true;
}

void NouCicle() {
  text1 = " ";
  for (int i = 0; i < NumeroEscenes * 2; i++) {
    if (!verificarSequenciaTemps(i)) {
      text1 = "Error seqüencia";
      Serial.println(F("Error sequencia temporal"));
      return;
    }
  }

  EstatAntic = EstatActual;
  EstatActual = 0;

  Serial.print(F("Inici Nou Cicle. Estat inicial = "));
  Serial.print(EstatActual);
  Serial.print(F(" MusicaMP3 = "));
  Serial.println(MusicaMP3);

  if (PrepararReproductorSiCal()) {
    miReproductor->reproducir(MusicaMP3);
  }

  for (int i = 0; i < numeroCanals; i++) {
    actualizarCanalFix(i, EstatActual);
  }

  referenciaTempsCicle = referenciaTempsEstat = referenciaTempsTransicio = micros();
  tempsActualCicle = tempsActualEstat = tempsActualTransicio = 0;
  contadorPuntTransicio = 0;

  V[10] = EstatActual + 1;
}

void PararReproduccio() {
  text1 = "  ";
  EstatPlay = "Stop";
  V[12] = 0;
  V[13] = 0;
  V[14] = 0;
  V[10] = 0;
  cicloEnCurso = false;

  miReproductor->detener();

  EstatActual = 0;
  EstatAntic = 0;
  contadorPuntTransicio = 0;

  for (int i = 0; i < numeroCanals; i++) {
    actualizarCanalFix(i, EstatActual);
  }

  tempsActualEstat = tempsActualCicle = tempsActualTransicio = 0;
  referenciaTempsEstat = referenciaTempsCicle = referenciaTempsTransicio = micros();

  Serial.println(F("Parada reproducció"));
}

void cridaTransicio() {
  const uint16_t t_pct = (uint16_t)(((uint32_t)contadorPuntTransicio * 1000UL) / numeroPuntsTransicio);
  for (int i = 0; i < numeroCanals; i++) {
    actualizarCanalTransicio(i, EstatActual, t_pct);
  }
}

void CanviEstat() {
  if (EstatActual == num_periodes - 1) {
    if (EstatSelector == 1 || EstatSelector == 7) {
      NouCicle();
    } else if (EstatSelector == 2) {
      PararReproduccio();
      triggerAnterior = digitalRead(TRIGGER_PIN);
      Serial.println(F("Cicle complet finalitzat. Esperant nou flanc de trigger."));
    }
    return;
  }

  EstatAntic = EstatActual;
  EstatActual++;

  referenciaTempsEstat = referenciaTempsTransicio = micros();
  tempsActualEstat = tempsActualTransicio = 0;
  contadorPuntTransicio = 0;

  V[10] = EstatActual + 1;

  if (EstatActual % 2 == 0) {
    for (int i = 0; i < numeroCanals; i++) {
      actualizarCanalFix(i, EstatActual);
    }
  } else {
    numeroPuntsTransicio = Temps[EstatActual] / tempsCiclesTransicio;
  }
}

void AvancarCicleSiCal() {
  tempsActualEstat = micros() - referenciaTempsEstat;
  tempsActualCicle = micros() - referenciaTempsCicle;
  tempsActualTransicio = micros() - referenciaTempsTransicio;
  V[14] = tempsActualCicle / 1000000;

  for (int i = 0; i < 8; i++) {
    if (tempsActualEstat >= Temps[i] && EstatActual == i) {
      CanviEstat();
    }
  }

  if (EstatActual % 2 == 1 && tempsActualTransicio >= tempsCiclesTransicio && NumeroEscenes != 1) {
    tempsActualTransicio = 0;
    referenciaTempsTransicio = micros();
    contadorPuntTransicio++;
    if (contadorPuntTransicio <= numeroPuntsTransicio) {
      cridaTransicio();
    }
  }
}

void PausarReproduccio() {
  text1 = ">> Pausa <<";
  EstatPlay = "Pausa";
  miReproductor->pausar();
  pausaCicleInici = micros();
}

void ContinuarReproduccio() {
  text1 = ">> reproduint <<";
  EstatPlay = "Play";
  miReproductor->reanudar();

  const uint32_t pausaDuracio = micros() - pausaCicleInici;
  referenciaTempsEstat += pausaDuracio;
  referenciaTempsCicle += pausaDuracio;
  referenciaTempsTransicio += pausaDuracio;
}

void IniciarReproduccio() {
  if (V[12] == 0) return;
  text1 = ">> reproduint <<";
  EstatPlay = "Play";
  V[13] = 0;
  cicloEnCurso = true;
  NouCicle();
}

void Reproduint() {
  text1 = "... reproduint ...";
}

void GestioCicles() {
  if (V[12] == 1 && EstatPlay == "Stop") IniciarReproduccio();
  if (V[12] == 0 && (EstatPlay == "Play" || EstatPlay == "Pausa")) PararReproduccio();
  if (V[13] == 1 && EstatPlay == "Play") PausarReproduccio();
  if (V[12] == 1 && V[13] == 0 && EstatPlay == "Pausa") ContinuarReproduccio();
  if (V[12] == 1 && EstatPlay == "Play") Reproduint();
}

void GravacioIntervals() {
  for (int i = 0; i < NumeroEscenes * 2; i++) {
    if (V[i + 21] != TempsAcumulat[i]) {
      // ajustarSequenciaPosteriors: desplaça els punts posteriors la
      // mateixa diferència, per conservar la seva durada relativa.
      const long diferencia = (long)V[i + 21] - (long)TempsAcumulat[i];
      for (int j = i + 1; j < NumeroEscenes * 2; j++) V[j + 21] += diferencia;

      if (verificarSequenciaTemps(i)) {
        for (int j = 0; j < NumeroEscenes * 2; j++) {
          TempsAcumulat[j] = V[j + 21];
          if (j == 0) {
            Parametres.tempsPeriodes[j] = Temps[j] = (uint32_t)TempsAcumulat[j] * 1000000UL;
          } else {
            Parametres.tempsPeriodes[j] = Temps[j] =
                (uint32_t)(TempsAcumulat[j] - TempsAcumulat[j - 1]) * 1000000UL;
          }
        }

        text1 = " ";
        tiempoTotalCiclo = 0;
        for (int k = 0; k < NumeroEscenes * 2; k++) tiempoTotalCiclo += Temps[k];
        V[15] = tiempoTotalCiclo / 1000000;

        markParamsDirty();
      } else {
        V[i + 21] = TempsAcumulat[i];
        text1 = "Error seqüencia";
      }
    }
  }
}

void Cicle() {
  GravacioIntervals();
  GestioCicles();
}

void actualitzarEstatEscenes() {
  V[18] = NumeroEscenes = Parametres.NumeroEscenes;
  num_periodes = NumeroEscenes * 2;
  EscenaActiva = Parametres.EscenaActiva;
}

// Inicialitza tots els canals des de la NVS ja carregada a canalsData[] i
// aplica l'escena actual (0 = primera escena).
void inicializarCanals() {
  apagarTotsElsCanals();
  for (int i = 0; i < numeroCanals; i++) {
    actualizarCanalFix(i, 0);
  }
}

void aplicarSelector(int estat) {
  for (int i = 0; i < numeroCanals; i++) {
    actualizarCanalFix(i, estat);
  }
}

void RecuperarValorsCanals() {
  V[1] = canalsData[Canal_1 - 1].valors[EscenaActiva - 1];
  V[2] = canalsData[Canal_2 - 1].valors[EscenaActiva - 1];
  V[3] = canalsData[Canal_3 - 1].valors[EscenaActiva - 1];

  enviarCanalEscena(Canal_1 - 1, EscenaActiva - 1);
  enviarCanalEscena(Canal_2 - 1, EscenaActiva - 1);
  enviarCanalEscena(Canal_3 - 1, EscenaActiva - 1);
}

void InicialitzarPrograma() {
  Serial.println(F("Inicialitzar el programa"));

  numeroCanals = constrain(Parametres.NumeroCanals, 1, MAX_CANALS);
  V[40] = numeroCanals;
  V[39] = MAX_CANALS;

  V[18] = NumeroEscenes = Parametres.NumeroEscenes;
  actualitzarEstatEscenes();

  V[16] = Parametres.NivellVolum;
  NivellVolum = Parametres.NivellVolum;
  miReproductor->ajustarVolumen(NivellVolum);

  V[0] = Parametres.NumeroMusica;
  MusicaMP3 = Parametres.NumeroMusica;

  for (int i = 0; i < 8; i++) Temps[i] = Parametres.tempsPeriodes[i];

  tiempoTotalCiclo = 0;
  for (int i = 0; i < NumeroEscenes * 2; i++) tiempoTotalCiclo += Temps[i];
  V[15] = tiempoTotalCiclo / 1000000;

  V[21] = TempsAcumulat[0] = Temps[0] / 1000000;
  for (int i = 1; i < 8; i++) {
    V[i + 21] = TempsAcumulat[i] = TempsAcumulat[i - 1] + Temps[i] / 1000000;
  }

  inicializarCanals();

  V[9] = EscenaActiva = 1;
  V[4] = Canal_1 = 1;
  V[5] = Canal_2 = 2;
  V[6] = Canal_3 = 3;

  RecuperarValorsCanals();

  V[12] = 0;
  V[13] = 0;
  V[41] = 0;
  V[42] = 0;
}

void Escenes() {
  if (V[35] != 0) {
    V[9] = EscenaActiva + V[35];
    if (V[9] < 1) V[9] = 1;
    // NumeroEscenes (no 4 fix): no es pot avançar més enllà de l'última
    // escena activa configurada.
    if (V[9] > NumeroEscenes) V[9] = NumeroEscenes;

    EscenaActiva = V[9];
    Parametres.EscenaActiva = EscenaActiva;

    RecuperarValorsCanals();

    const int estatCorrespondent = (EscenaActiva - 1) * 2;
    aplicarSelector(estatCorrespondent);

    V[35] = 0;
    return;
  }

  if (V[7] != 0) {
    if (Canal_1 == 1 && V[7] == -1) return;
    if (Canal_3 >= numeroCanals && V[7] == 1) return;

    Canal_1 += 3 * (int)V[7];
    Canal_2 += 3 * (int)V[7];
    Canal_3 += 3 * (int)V[7];
    V[4] = Canal_1;
    V[5] = Canal_2;
    V[6] = Canal_3;

    RecuperarValorsCanals();
    V[7] = 0;
    return;
  }

  if (V[1] != canalsData[Canal_1 - 1].valors[EscenaActiva - 1]) {
    guardarEnviarValor(Canal_1 - 1, EscenaActiva - 1, (int)V[1]);
    return;
  }
  if (V[2] != canalsData[Canal_2 - 1].valors[EscenaActiva - 1]) {
    guardarEnviarValor(Canal_2 - 1, EscenaActiva - 1, (int)V[2]);
    return;
  }
  if (V[3] != canalsData[Canal_3 - 1].valors[EscenaActiva - 1]) {
    guardarEnviarValor(Canal_3 - 1, EscenaActiva - 1, (int)V[3]);
    return;
  }
}

void AjustVolum() {
  if (V[16] != NivellVolum) {
    NivellVolum = Parametres.NivellVolum = (int)V[16];
    markParamsDirty();
    miReproductor->ajustarVolumen(NivellVolum);
  }
}

void performFactoryReset() {
  Parametres.EscenaActiva = 1;
  Parametres.NumeroEscenes = 4;
  Parametres.NumeroMusica = 0;
  Parametres.NivellVolum = 20;
  for (int i = 0; i < 8; i++) Parametres.tempsPeriodes[i] = 5000000UL;
  Parametres.NumeroCanals = DEFAULT_NUMERO_CANALS;
  saveParams();

  // memset ja deixa cada canal a valors=0 i transicions=LINEAL/0% (el
  // primer TipusTransicio de l'enum és LINEAL=0), no cal cap bucle a part.
  memset(canalsData, 0, sizeof(canalsData));
  for (int i = 0; i < CHANNEL_BUFFER_SIZE; i++) {
    valorActual[i] = 0;
  }
  markAllCanalsDirty();
  saveCanals();

  memset(channelNames, 0, sizeof(channelNames));
  pessebeName = "";
  descripcio = "";
  markAllNamesDirty();
  saveNames();

  Serial.println(F("Reset de fàbrica complet."));

  InicialitzarPrograma();
}

void ConfiguracioParametres() {
  if (V[18] != NumeroEscenes) {
    Parametres.EscenaActiva = 1;
    for (int i = 0; i < 8; i++) Parametres.tempsPeriodes[i] = 5000000UL;
    NumeroEscenes = Parametres.NumeroEscenes = (int)V[18];

    actualitzarEstatEscenes();
    markParamsDirty();
    InicialitzarPrograma();
  }

  if (V[0] != MusicaMP3) {
    MusicaMP3 = Parametres.NumeroMusica = (int)V[0];
    markParamsDirty();
  }

  const int valorEntrada = constrain((int)V[40], 1, MAX_CANALS);
  int novesCanals = valorEntrada;
  if (novesCanals % 3 != 0) {
    novesCanals = ((novesCanals / 3) + 1) * 3;
    if (novesCanals > MAX_CANALS) novesCanals = MAX_CANALS;
  }
  if (novesCanals != numeroCanals) {
    numeroCanals = Parametres.NumeroCanals = novesCanals;
    V[40] = numeroCanals;
    markParamsDirty();
    InicialitzarPrograma();
  }

  if (V[41] == 1 && V[42] == 1) {
    performFactoryReset();
    V[41] = V[42] = 0;
  }
}

// ---------------------------------------------------------------------------
// Protocol `!Vxx=valor$` / `!Vxx=?$`
// ---------------------------------------------------------------------------

// Envia una trama completa per BLE, fragmentada en trossos de com a màxim
// el MTU negociat amb el client actual — mateixa lògica que
// ardmx-one-firmware's pròpia sendFrame(), vegeu el comentari de capçalera
// d'aquest fitxer.
void sendFrame(const String &frame) {
  if (!bleClientConnected || bleNotifyCharacteristic == nullptr) return;

  const uint16_t mtu = bleServer != nullptr
      ? bleServer->getPeerMTU(bleConnHandle)
      : 0;
  const size_t chunkSize = (mtu > 23) ? (size_t)(mtu - 3) : 20;

  const size_t len = frame.length();
  size_t offset = 0;
  while (offset < len) {
    const size_t n = min(len - offset, chunkSize);
    bleNotifyCharacteristic->setValue(
        (const uint8_t *)frame.c_str() + offset, n);
    bleNotifyCharacteristic->notify();
    offset += n;
  }
}

void replyNumber(int index, float value) {
  String frame = "!V";
  if (index < 10) frame += '0';
  frame += index;
  frame += '=';
  // String(value, 2): mateix format que SerialBT.print(float) feia servir
  // per defecte (2 decimals) — es manté igual per no canviar res del que
  // l'app espera rebre.
  frame += String(value, 2);
  frame += '$';
  sendFrame(frame);
}

void replyText(int index, const char *text) {
  String frame = "!V";
  if (index < 10) frame += '0';
  frame += index;
  frame += '=';
  frame += text;
  frame += '$';
  sendFrame(frame);
}

void handleNameChange(const String &rawInput) {
  const String clean = sanitizeName(rawInput);
  if (clean.length() == 0) return;

  prefs.begin("ardmxevo", false);
  prefs.putString("btname", clean);
  prefs.end();

  btDeviceName = clean;
  replyText(63, btDeviceName.c_str());

  delay(200);
  ESP.restart();
}

// V73: l'app hi envia el PIN per autenticar-se just després de connectar.
// Sempre es processa (encara que ja calgui PIN i no s'hagi enviat encara),
// és precisament l'única manera d'arribar a autenticar-se.
void handlePinVerify(const String &rawInput) {
  const String attempt = sanitizePin(rawInput);
  pinAuthenticated = storedPin.length() > 0 && attempt == storedPin;
  replyText(73, pinAuthenticated ? "OK" : "ERROR");
}

// V74: posar/canviar el PIN — només té efecte si ja estàs autenticat (o si
// encara no hi havia cap PIN, cas en què "autenticat" ja és cert per
// definició, vegeu la comprovació al capdamunt de processFrame()).
void handlePinSet(const String &rawInput) {
  const String clean = sanitizePin(rawInput);
  if (clean.length() == 0) return;

  prefs.begin("ardmxevo", false);
  prefs.putString("pin", clean);
  prefs.end();

  storedPin = clean;
  replyText(74, "OK");
}

// V75: restableix el PIN (torna a "sense PIN"). Sempre s'accepta,
// independentment de pinAuthenticated — vegeu el comentari de storedPin
// sobre per què això és acceptable per l'amenaça real que es vol evitar.
void handlePinReset(const String &rawInput) {
  prefs.begin("ardmxevo", false);
  prefs.remove("pin");
  prefs.end();

  storedPin = "";
  pinAuthenticated = false;
  replyText(75, "OK");
}

void handleChannelNameChange(int index, const String &rawInput) {
  const int slot = index - 65;
  const int channel = (slot == 0) ? Canal_1 : (slot == 1) ? Canal_2 : Canal_3;
  const String clean = sanitizeText(rawInput, MAX_CHANNEL_NAME_LENGTH);
  clean.toCharArray(channelNames[channel - 1], MAX_CHANNEL_NAME_LENGTH + 1);
  markNamesDirty(channel - 1);
  replyText(index, channelNames[channel - 1]);
}

void handlePessebeNameChange(const String &rawInput) {
  pessebeName = sanitizeText(rawInput, MAX_PESSEBE_NAME_LENGTH);
  markNamesDirty();
  replyText(68, pessebeName.c_str());
}

void handleDescriptionChange(const String &rawInput) {
  descripcio = sanitizeText(rawInput, MAX_DESCRIPTION_LENGTH);
  markNamesDirty();
  replyText(69, descripcio.c_str());
}

// V71: consulta o assignació de les 4 escenes I les 4 transicions PRÒPIES
// d'UN canal explícit. V72 (transicions globals) ha desaparegut — cada
// canal porta ara les seves pròpies, incloses aquí.
//   V71=N                                          -> consulta
//   V71=N|v1|v2|v3|v4|t1|s1|t2|s2|t3|s3|t4|s4|nom  -> assigna
// Resposta (en tots dos casos): "v1|v2|v3|v4|t1|s1|t2|s2|t3|s3|t4|s4|nom"
void handleChannelBulk(const String &rawInput) {
  const int firstPipe = rawInput.indexOf('|');
  const int channel = constrain(
      (firstPipe == -1 ? rawInput : rawInput.substring(0, firstPipe)).toInt(), 1, numeroCanals);

  if (firstPipe != -1) {
    String rest = rawInput.substring(firstPipe + 1);
    long fields[12];
    for (int i = 0; i < 12; i++) {
      const int p = rest.indexOf('|');
      if (p == -1) {
        fields[i] = rest.toInt();
        rest = "";
      } else {
        fields[i] = rest.substring(0, p).toInt();
        rest = rest.substring(p + 1);
      }
    }
    const String nom = sanitizeText(rest, MAX_CHANNEL_NAME_LENGTH);

    CanalData &c = canalsData[channel - 1];
    for (int i = 0; i < 4; i++) {
      c.valors[i] = (uint8_t)constrain(fields[i], 0, 255);
    }
    for (int i = 0; i < 4; i++) {
      c.transicions[i].tipus = (TipusTransicio)constrain(fields[4 + i * 2], 0, 3);
      c.transicions[i].saltPercent = (uint8_t)constrain(fields[4 + i * 2 + 1], 0, 100);
    }
    if (EscenaActiva >= 1 && EscenaActiva <= 4) {
      valorActual[channel - 1] = c.valors[EscenaActiva - 1];
    }
    nom.toCharArray(channelNames[channel - 1], MAX_CHANNEL_NAME_LENGTH + 1);

    markCanalsDirty(channel - 1);
    markNamesDirty(channel - 1);

    // Si el canal assignat és un dels 3 actualment seleccionats als
    // sliders (Canal_1/2/3), V[1-3] queda desactualitzat — i Escenes(), que
    // segueix corrent contínuament mentre V[50]==4 (aquesta pantalla hi és
    // mapejada), detectaria V[1] com "l'usuari ha mogut l'slider" i
    // sobreescriuria el valor que acabem d'importar amb l'antic. Mateix bug
    // (i mateixa solució) que ARDMX4.ino's handleChannelBulk() amb V63.
    RecuperarValorsCanals();
  }

  const CanalData &c = canalsData[channel - 1];
  String reply = String(c.valors[0]) + "|" + String(c.valors[1]) + "|" +
                 String(c.valors[2]) + "|" + String(c.valors[3]);
  for (int i = 0; i < 4; i++) {
    reply += "|" + String((int)c.transicions[i].tipus) + "|" + String(c.transicions[i].saltPercent);
  }
  reply += "|" + String(channelNames[channel - 1]);
  replyText(71, reply.c_str());
}

void handleWrite(int index, float value) {
  if (index < 0 || index >= V_SIZE) return;
  V[index] = value;
}

void handleRequest(int index) {
  if (index >= 0 && index < V_SIZE) {
    replyNumber(index, V[index]);
    return;
  }
  switch (index) {
    case 62:
      replyText(62, FIRMWARE_VERSION_TEXT);
      break;
    case 63:
      replyText(63, btDeviceName.c_str());
      break;
    case 64:
      replyText(64, buildIdentifyJson().c_str());
      break;
    case 65:
    case 66:
    case 67: {
      const int slot = index - 65;
      const int channel = (slot == 0) ? Canal_1 : (slot == 1) ? Canal_2 : Canal_3;
      replyText(index, channelNames[channel - 1]);
      break;
    }
    case 68:
      replyText(68, pessebeName.c_str());
      break;
    case 69:
      replyText(69, descripcio.c_str());
      break;
    case 76:
      // Índex separat del 74 (que és l'ACK "OK"/"ERROR" d'activar el PIN) a
      // propòsit: si es reutilitzés el 74, l'app rebria l'ACK de la
      // desada com si fos el valor del PIN. Només es pot arribar aquí ja
      // autenticat (o sense PIN) — vegeu el "gated" a processFrame() —
      // així que ensenyar-lo no exposa res que qui pregunta no hagi pogut
      // demostrar ja que sap.
      replyText(76, storedPin.c_str());
      break;
    default:
      break;
  }
}

void processFrame(const String &body) {
  if (body.length() < 2 || body[0] != 'V') return;
  const int eq = body.indexOf('=');
  if (eq < 2) return;

  const int index = body.substring(1, eq).toInt();
  const String rhs = body.substring(eq + 1);

  // Mentre calgui PIN i encara no s'hagi enviat el correcte, només es
  // processen V64 (identificació — cal per saber que fa falta PIN), V73
  // (verificar-lo) i V75 (restablir-lo) — tota la resta es descarta en
  // silenci, tant lectures (més avall) com escriptures.
  const bool gated = storedPin.length() > 0 && !pinAuthenticated;
  if (gated && index != 64 && index != 73 && index != 75) return;

  if (rhs == "?") {
    handleRequest(index);
    return;
  }

  if (index == 63) {
    handleNameChange(rhs);
  } else if (index >= 65 && index <= 67) {
    handleChannelNameChange(index, rhs);
  } else if (index == 68) {
    handlePessebeNameChange(rhs);
  } else if (index == 69) {
    handleDescriptionChange(rhs);
  } else if (index == 71) {
    handleChannelBulk(rhs);
  } else if (index == 73) {
    handlePinVerify(rhs);
  } else if (index == 74) {
    handlePinSet(rhs);
  } else if (index == 75) {
    handlePinReset(rhs);
  } else {
    handleWrite(index, rhs.toFloat());
  }
}

// Alimenta UN byte a l'acumulador de trames — abans el cos del while() de
// pollBluetooth(), ara cridat des de drainBleRxQueue() per cada byte rebut
// per BLE.
void feedByte(char c) {
  if (c == '!') {
    btFrameBuffer = "";
  } else if (c == '$') {
    processFrame(btFrameBuffer);
    btFrameBuffer = "";
  } else {
    btFrameBuffer += c;
    if (btFrameBuffer.length() > 512) btFrameBuffer = "";
  }
}

// Buida la cua de trossos rebuts per BLE (omplerta pel callback onWrite(),
// que corre a la tasca de l'stack BLE, no a loop()) i alimenta cada byte a
// feedByte() — aquí, dins loop(), és on realment es crida processFrame()/
// handleWrite() i es toca tot l'estat global (V[], canalsData...), mai
// directament des del callback (vegeu el comentari de capçalera
// "Concurrència").
void drainBleRxQueue() {
  BleRxChunk chunk;
  while (xQueueReceive(bleRxQueue, &chunk, 0) == pdTRUE) {
    for (size_t i = 0; i < chunk.length; i++) {
      feedByte((char)chunk.data[i]);
    }
  }
}

// ---------------------------------------------------------------------------
// Callbacks de NimBLE — s'executen a la tasca de l'stack BLE, no a loop().
// NimBLE-Arduino 1.4.x fa servir ble_gap_conn_desc* en aquests callbacks
// (no NimBLEConnInfo&, que és de la 2.x) — confirmat compilant contra la
// versió que resol aquest platformio.ini, mateixa troballa que a
// ardmx-one-firmware.
// ---------------------------------------------------------------------------

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
    bleClientConnected = true;
    bleConnHandle = desc->conn_handle;
    pinAuthenticated = false;
  }

  void onDisconnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
    bleClientConnected = false;
    bleConnHandle = BLE_HS_CONN_HANDLE_NONE;
    NimBLEDevice::startAdvertising();
  }
};

class WriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic,
               ble_gap_conn_desc *desc) override {
    const std::string value = characteristic->getValue();
    size_t offset = 0;
    while (offset < value.size()) {
      BleRxChunk chunk;
      chunk.length = min(value.size() - offset, BLE_RX_CHUNK_MAX);
      memcpy(chunk.data, value.data() + offset, chunk.length);
      xQueueSend(bleRxQueue, &chunk, 0);
      offset += chunk.length;
    }
  }
};

void bleInit() {
  NimBLEDevice::init(btDeviceName.c_str());
  NimBLEDevice::setMTU(BLE_PREFERRED_MTU);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService *service = bleServer->createService(BLE_SERVICE_UUID);

  NimBLECharacteristic *writeCharacteristic = service->createCharacteristic(
      BLE_WRITE_CHAR_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  writeCharacteristic->setCallbacks(new WriteCallbacks());

  bleNotifyCharacteristic = service->createCharacteristic(
      BLE_NOTIFY_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  service->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->start();
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  memset(V, 0, sizeof(V));

  loadParams();
  loadCanals();
  loadNames();
  loadBtName();
  loadPin();

  dmxInit();
  dfSerial.begin(9600, SERIAL_8N1, /*RX=*/16, /*TX=*/17);
  miReproductor = new Reproductor(dfSerial);

  bleRxQueue = xQueueCreate(8, sizeof(BleRxChunk));
  bleInit();

  InicialitzarPrograma();

  V[11] = Parametres.EstatSelector;
  EstatSelector = Parametres.EstatSelector;

  switch (EstatSelector) {
    case 1:
      NouCicle();
      break;
    case 2:
      aplicarSelector(1);
      break;
    case 3:
      aplicarSelector(1);
      break;
    case 4:
      aplicarSelector(2);
      break;
    case 5:
      aplicarSelector(4);
      break;
    case 6:
      aplicarSelector(6);
      break;
    case 7:
      RecuperarValorsCanals();
      break;
    default:
      break;
  }

  V[41] = 0;
  actualitzarModeLed();

  Serial.println(F("ARDMX EVO iniciat"));
}

void loop() {
  drainBleRxQueue();
  AjustVolum();
  GestioDFPlayer();
  actualitzarModeLed();
  updateStatusLed();

  // Detectar canvi de selector principal (V11)
  if (V[11] != EstatSelector) {
    Parametres.EstatSelector = EstatSelector = (int)V[11];
    V[10] = 0;

    PararReproduccio();
    markParamsDirty();

    switch (EstatSelector) {
      case 1:
        NouCicle();
        break;
      case 2:
        aplicarSelector(1);
        break;
      case 3:
        aplicarSelector(1);
        break;
      case 4:
        aplicarSelector(2);
        break;
      case 5:
        aplicarSelector(4);
        break;
      case 6:
        aplicarSelector(6);
        break;
      case 7:
        RecuperarValorsCanals();
        break;
      default:
        break;
    }
  }

  if (EstatSelector == 1 && NumeroEscenes == 1) {
    V[11] = 3;
  }

  if (EstatSelector == 1 && NumeroEscenes != 1) {
    AvancarCicleSiCal();
  }

  // EstatSelector == 2: trigger extern
  if (EstatSelector == 2) {
    const bool triggerActual = digitalRead(TRIGGER_PIN);
    if (!triggerActual && triggerAnterior && !cicloEnCurso) {
      cicloEnCurso = true;
      NouCicle();
    }
    triggerAnterior = triggerActual;

    if (cicloEnCurso) {
      AvancarCicleSiCal();
    }
  }

  // EstatSelector == 7: pantalles de configuració/escenes/cicle de l'app
  if (EstatSelector == 7) {
    if (V[50] == 3 && (EstatPlay == "Play" || EstatPlay == "Pausa")) PararReproduccio();
    if (V[50] == 4) ConfiguracioParametres();
    if (V[50] == 1) Cicle();
    if (V[50] == 5) Escenes();

    if (cicloEnCurso && EstatPlay == "Play") {
      AvancarCicleSiCal();
    }
  }

  // Desats amb debounce — com a màxim UN per volta (else if): cada desat ja
  // bloqueja loop()/DMX uns ms mentre escriu a flash, i com que
  // paramsDirty/canalsDirty/namesDirty comparteixen el mateix
  // lastChangeMillis, més d'un podia arribar dirty alhora (p.ex. V71 marca
  // canalsDirty i namesDirty juntes) i apilar-se en un sol bloqueig llarg
  // dins la mateixa volta — mateix motiu que a ardmx-one-firmware.
  if (paramsDirty && millis() - lastChangeMillis > SAVE_DEBOUNCE_MS) {
    saveParams();
  } else if (canalsDirty && millis() - lastChangeMillis > SAVE_DEBOUNCE_MS) {
    saveCanals();
  } else if (namesDirty && millis() - lastChangeMillis > SAVE_DEBOUNCE_MS) {
    saveNames();
  }

  // Enviament DMX limitat en freqüència
  static uint32_t lastDmxSendMillis = 0;
  const uint32_t now = millis();
  if (now - lastDmxSendMillis >= DMX_SEND_INTERVAL_MS) {
    dmxSendFrame();
    lastDmxSendMillis = now;
  }
}
