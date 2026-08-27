# ARDMX EVO — firmware ESP32

Versió actual: **v2.0** (`FIRMWARE_VERSION_TEXT`, resposta a V62 —
comprovable des de l'app un cop connectat, a Crèdits o Menú Principal).

Equivalent funcional de l'[ARDMX4](https://github.com/xaviermila-png/ardmx4-firmware)
(Arduino Mega) amb un cor **ESP32**, reduint cost i afegint Bluetooth i persistència
integrats. Mateixa funcionalitat: 4 escenes DMX amb transicions **pròpies de cada canal**
(tipus Lineal/Salt/Ease In/Ease Out + percentatge de salt, no compartides entre canals),
cicle sincronitzat amb música MP3, i control des d'un mòbil per Bluetooth Low Energy (BLE)
mitjançant el protocol `!Vxx=val$` (framework Virtuino).

## Maquinari
- ESP32 DevKit V1 (WROOM-32), Bluetooth Low Energy (BLE/GATT — migrat de Bluetooth
  Classic/SPP el 2026-08, iOS no permet Classic a apps de tercers sense certificació MFi)
- MAX485 amb direcció automàtica per maquinari (DMX per UART, `DMX_NUM_1`)
- DFPlayer Mini per UART1 (nivells adaptats: divisor de tensió a la línia RX de l'ESP32)
- Trigger extern (mode Manual) a GPIO4, `INPUT_PULLUP`
- Persistència NVS (partició integrada de l'ESP32, sense EEPROM externa, ampliada a 64 KB)

## Relació amb els altres firmwares del projecte
- [`ardmx4-firmware`](https://github.com/xaviermila-png/ardmx4-firmware) — Arduino Mega,
  font de la veritat funcional (lògica de cicle/escenes/transicions i mapa `V[]` base, que
  aquest firmware reutilitza tal qual per compatibilitat de protocol).
- [`ardmx-one-firmware`](https://github.com/xaviermila-png/ardmx-one-firmware) — ESP32,
  primer firmware d'aquesta família nova; d'aquí es reutilitzen les lliçons de maquinari
  real (mida de pila del loop task, `DMX_NUM_1` en lloc de `DMX_NUM_2`, límit de freqüència
  d'enviament DMX, persistència NVS en trossos petits amb comprovació de retorn, handshake
  d'identificació de dispositiu).

## Instal·lar el firmware sense compilar

Dues carpetes llestes per flashejar un ESP32 amb el firmware ja compilat, sense
haver d'obrir el projecte ni compilar res (mateix patró que
[`ardmx-one-firmware`](https://github.com/xaviermila-png/ardmx-one-firmware)):

- **`installer_standalone/`** (recomanada) — completament autònoma, amb el seu
  propi `esptool.exe` (binari oficial, sense dependències). Es pot copiar la
  carpeta sencera a qualsevol PC amb Windows, encara que no hi hagi Python ni
  PlatformIO instal·lats.
- **`installer/`** — versió lleugera que reutilitza el Python/`esptool.py` que
  ja porta PlatformIO instal·lat en aquest ordinador.

Dona doble clic a `install_ardmx_evo.bat`, escriu el port COM de l'ESP32 i
prem Enter.

**Què fa exactament** — a diferència de l'ARDMX One, aquest firmware **no fa
servir OTA** (només té una partició `factory`, vegeu `partitions.csv`), així
que l'instal·lador escriu només 3 blocs, sense `boot_app0.bin`:

| Fitxer | Adreça | Contingut |
|---|---|---|
| `bootloader.bin` | `0x1000` | Bootloader de l'ESP32 |
| `partitions.bin` | `0x8000` | Taula de particions |
| `firmware.bin` | `0x20000` | El programa (nota: no `0x10000` — aquest projecte mou l'aplicació per fer lloc a la NVS ampliada, vegeu `board_upload.offset_address` a `platformio.ini`) |

La NVS (`0x10000`-`0x1FFFF`, 64 KB) mai es toca — reflashejar amb aquest
instal·lador no esborra la configuració desada. **Important**: no incloure mai
`boot_app0.bin` en cap flash d'aquest projecte — vegeu el bug documentat més
avall (ara resolt movent la NVS, no calia deixar de flashejar-lo enlloc més).

**Per actualitzar els binaris de l'instal·lador** quan canvia el codi font:
`pio run` per compilar, i copiar `.pio/build/esp32dev/bootloader.bin`,
`.pio/build/esp32dev/partitions.bin` i `.pio/build/esp32dev/firmware.bin` a la
carpeta `bin/` de totes dues instal·ladors.

## Bugs trobats i corregits

### `pio run -t upload` escriu `boot_app0.bin` a sobre de la NVS — RESOLT

**Símptoma real, confirmat en maquinari el 2026-08-24** (a `ardmx-one-firmware`,
que patia exactament el mateix disseny de partició): després de configurar
valors/noms/transicions de canal i reiniciar la placa, tot tornava a zero.
Descobert originalment per casualitat el 2026-08-17 mentre es preparava
l'instal·lador `.bat`, en llegir amb detall la comanda `esptool` exacta que
genera PlatformIO — en aquell moment encara no s'havia confirmat l'impacte
real.

**Causa**: aquest projecte no fa servir OTA — la `partitions.csv` anterior
només definia `nvs` (`0x9000`, ampliada a `0x17000` = 92 KB) i `app0` tipus
`factory` (`0x20000`). Però l'entorn `esp32dev` de PlatformIO afegeix per
defecte un quart fitxer a **qualsevol** comanda d'`upload` (fet des de
PlatformIO/VS Code, NO des dels instal·ladors `.bat`), `boot_app0.bin`,
escrit sempre a `0xE000` — una adreça pensada per a la partició `otadata`
que els projectes AMB OTA tenen normalment buida just entre la NVS petita
per defecte (20 KB, acaba a `0xDFFF`) i el propi `otadata`. Com que aquí la
NVS s'havia ampliat fins a `0x1FFFF`, `0xE000` queia **dins** la NVS en lloc
d'en un espai buit, i cada `pio run -t upload` hi escrivia 8 KB de dades
irrellevants a sobre — esborrant qualsevol clau NVS que hi tingués dades
emmagatzemades.

**Fix aplicat (2026-08-24)**: la NVS ja NO comença a `0x9000` — ara comença
just DESPRÉS de la zona `0xE000`-`0xFFFF` (a `0x10000`, 64 KB), deixant
`0x9000`-`0xDFFF` sense reclamar (20 KB perduts, irrellevant amb 4 MB de
flash). `app0` es queda igual, a `0x20000`. Vegeu `partitions.csv` actual.
Amb aquest canvi, `pio run -t upload` pot seguir escrivint `boot_app0.bin` a
`0xE000` amb tota tranquil·litat — ja no hi ha res important allà.

**Lliçó per als altres firmwares de la família**: qualsevol projecte que
ampliï la NVS més enllà del que la partició `otadata` per defecte ocupa
(`0xE000`-`0xFFFF`) ha de deixar aquesta franja sense reclamar, no donar per
fet que PlatformIO respecta la taula de particions personalitzada en
calcular què flashejar.

### Salt brusc de color a les transicions per sobre d'uns 100-110 canals actius

**Símptoma**: amb el nombre de canals actius (V40) per sobre d'uns 100-110, les
transicions de color entre escenes deixaven d'arribar suaument al valor final — als
últims instants abans de canviar d'escena, el color saltava de cop en lloc de continuar
la interpolació gradual. Amb pocs canals (com el límit conegut de ~40 a l'antic
`ardmx4-firmware`/Mega) semblava anar bé, però l'EVO (ESP32, molt més ràpid que el Mega)
també ho patia, només que a partir d'un llindar de canals molt més alt.

**Causa real**: `dmxSendFrame()` cridava `dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK)` just
després d'iniciar cada trama, bloquejant **tot** el `loop()` fins que la trama s'acabava
de transmetre físicament pel bus DMX. El temps de transmissió escala amb el nombre de
canals (a 250 kbit/s, ~44µs/byte) — a 510 canals arribava a ~23ms per trama. Com que
`dmxSendFrame()` es crida cada `DMX_SEND_INTERVAL_MS` (25ms) i els "tics" d'interpolació
de la transició van cada `tempsCiclesTransicio` (10ms), aquest bloqueig repetit robava una
fracció fixa i acumulativa del temps disponible per fer tics — no un bloqueig puntual gran,
sinó molts petits repartits per tota la transició. Per sobre d'un cert nombre de canals,
la fracció robada era prou gran perquè el comptador de tics no arribés al total esperat
abans que el rellotge de paret digués que tocava canviar d'escena, deixant el canal a mig
camí i saltant bruscament al valor final quan `actualizarCanalFix()` el fixava.

**Per què no era obvi**: `dmx_send_num()` (de la llibreria `esp_dmx`) ja fa internament una
espera curta (≤23ms) abans d'iniciar cada trama nova, esperant que l'ANTERIOR s'hagi acabat
de transmetre. La nostra pròpia crida explícita a `dmx_wait_sent()` immediatament després
era per tant **redundant** — no protegia de res que la llibreria no protegís ja, només
afegia un bloqueig extra fins que la trama ACTUAL (no l'anterior) acabava de sortir.

**Diagnosi**: instrumentació temporal amb `Serial` (durada de `loop()`/`dmxSendFrame()`,
tics completats vs. esperats per transició, forat màxim entre tics consecutius), provada
en maquinari real pujant el nombre de canals gradualment (21 → 510). Detall important: fer
`Serial.print()` en directe just en el moment crític (en acabar cada transició) esbiaixava
la pròpia mesura — calia diferir els prints fora del camí crític (guardar el resultat i
imprimir-lo des d'un altre punt del `loop()`, com a molt un cop per segon).

**Fix**: eliminar la crida a `dmx_wait_sent()` de `dmxSendFrame()` — `dmx_send_num()` ja
garanteix que les trames no se solapin. Validat en maquinari real (llums + música) fins al
màxim de 510 canals sense pèrdua de tics ni corrupció de la sortida DMX.

**Lliçó per als altres firmwares de la família** (`ardmx4-firmware`/Mega,
`ardmx-one-firmware`): revisar qualsevol crida bloquejant després d'iniciar un enviament
DMX/sèrie — pot limitar silenciosament el nombre de canals gestionables sense transicions
fluides, sense que hi hagi cap error ni crash evident que ho delati.

## Events (V77/V78) — específic de l'EVO

Fins a 10 accions programades ("events"), cadascuna disparada en un instant
concret del cicle (`moment`, segons des de l'inici) i que dura uns segons
(`durada`) abans de revertir-se. Cada event combina, independentment:

- **Un so puntual**: `player.advertise(pista)` reprodueix un fitxer de la
  carpeta `ADVERT/` del USB del DFPlayer — pausa la música de fons en curs
  i la reprèn tota sola en acabar (comportament natiu del mòdul, ja validat
  al sketch de prova aïllat `prova_so_extra`, branca no fusionada).
- **Un canal forçat a 255** durant la durada de l'event, per sobre del que
  digui l'escena/transició activa — `canalForcatPerEvent[]` bloqueja el
  recàlcul normal (`actualizarCanalFix`/`actualizarCanalTransicio`/
  `enviarCanalEscena`) mentre l'event el controla, i el restaura en acabar.

Cal com a mínim un dels dos (so o canal). Si dos events coincideixen sobre
el mateix canal o so, el més recent sempre pren el control; el primer no
es restaura en revertir si l'ha superat un segon.

`GestioEvents()` dispara/revertit els events des de la mateixa base de
temps que la resta del cicle (`tempsActualCicle`, cridat des
d'`AvancarCicleSiCal()`), un cop com a màxim per cicle. V78 permet
disparar un event manualment des de l'app ("Provar" a la pantalla Events),
amb el seu propi temporitzador basat en `millis()` perquè funcioni igual
amb el cicle actiu o aturat; en acabar la prova es fa un `stop()`
addicional del DFPlayer perquè no quedi sonant música de fons real quan no
n'hi ha cap cicle en marxa (el mòdul reprèn la darrera pista coneguda en
acabar l'advertise, independentment que hi hagi cap reproducció "de
veritat" en curs).

Protocol (mateix patró que V71, consulta/assignació per número
d'element):
```
V77=N                            -> consulta l'event N (0-9)
V77=N|moment|durada|pista|canal  -> assigna
V78=N                            -> dispara l'event N immediatament
```

## Exportació/importació de la configuració (des de l'app)
Aquest firmware no genera ni llegeix cap fitxer JSON — l'exportació/
importació des de la pantalla "Configuració" de l'app es munta i s'aplica
sencera a base de peticions d'aquest mateix protocol `!Vxx=valor$` (V71 per
canal, V18/V40/V21-28/V68/V69, V00/V16 pel so/volum, i V77 pels events).
L'esquema JSON unificat (vàlid tant per a l'ARDMX EVO com per a l'ARDMX One
v2, amb els camps de so/volum i els events agrupats en blocs opcionals que
el One v2 no té) es documenta a `ardmx_app`.

## Estat
Funcional i validat en maquinari real — BLE, cicle/escenes/transicions,
DFPlayer, events (V77/V78) i persistència NVS operatius. Vegeu "Bugs
trobats i corregits" per l'historial de problemes reals detectats i
corregits.
