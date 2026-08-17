# ARDMX4 EVO — firmware ESP32

Equivalent funcional de l'[ARDMX4](https://github.com/xaviermila-png/ardmx4-firmware)
(Arduino Mega) amb un cor **ESP32**, reduint cost i afegint Bluetooth i persistència
integrats. Mateixa funcionalitat: 4 escenes DMX amb transicions, cicle sincronitzat amb
música MP3, i control des d'un mòbil per Bluetooth mitjançant el protocol `!Vxx=val$`
(framework Virtuino).

## Maquinari
- ESP32 DevKit V1 (WROOM-32), Bluetooth clàssic (SPP)
- MAX485 amb direcció automàtica per maquinari (DMX per UART, `DMX_NUM_1`)
- DFPlayer Mini per UART1 (nivells adaptats: divisor de tensió a la línia RX de l'ESP32)
- Persistència NVS (partició integrada de l'ESP32, sense EEPROM externa)

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

Dona doble clic a `install_ardmx4_evo.bat`, escriu el port COM de l'ESP32 i
prem Enter.

**Què fa exactament** — a diferència de l'ARDMX One, aquest firmware **no fa
servir OTA** (només té una partició `factory`, vegeu `partitions.csv`), així
que l'instal·lador escriu només 3 blocs, sense `boot_app0.bin`:

| Fitxer | Adreça | Contingut |
|---|---|---|
| `bootloader.bin` | `0x1000` | Bootloader de l'ESP32 |
| `partitions.bin` | `0x8000` | Taula de particions |
| `firmware.bin` | `0x20000` | El programa (nota: no `0x10000` — aquest projecte mou l'aplicació per fer lloc a la NVS ampliada, vegeu `board_upload.offset_address` a `platformio.ini`) |

La NVS (`0x9000`-`0x1FFFF`, 92 KB) mai es toca — reflashejar amb aquest
instal·lador no esborra la configuració desada. **Important**: no incloure mai
`boot_app0.bin` en cap flash d'aquest projecte — vegeu el bug documentat més
avall, que és precisament sobre això.

**Per actualitzar els binaris de l'instal·lador** quan canvia el codi font:
`pio run` per compilar, i copiar `.pio/build/esp32dev/bootloader.bin`,
`.pio/build/esp32dev/partitions.bin` i `.pio/build/esp32dev/firmware.bin` a la
carpeta `bin/` de totes dues instal·ladors.

## Bugs trobats i corregits

### `pio run -t upload` escriu `boot_app0.bin` a sobre de la NVS

**Símptoma**: cap de moment (no reportat per l'usuari) — descobert per
casualitat el 2026-08-17 mentre es preparava l'instal·lador `.bat`, en llegir
amb detall la comanda `esptool` exacta que genera PlatformIO.

**Causa**: aquest projecte no fa servir OTA — `partitions.csv` només defineix
`nvs` (`0x9000`, ampliada a `0x17000` = 92 KB) i `app0` tipus `factory`
(`0x20000`). Però l'entorn `esp32dev` de PlatformIO afegeix per defecte un
quart fitxer a **qualsevol** comanda d'`upload`, `boot_app0.bin`, escrit
sempre a `0xE000` — una adreça pensada per a la partició `otadata` que els
projectes AMB OTA tenen normalment buida just entre la NVS petita per defecte
(20 KB, acaba a `0xDFFF`) i el propi `otadata`. Com que aquí la NVS s'ha
ampliat fins a `0x1FFFF` per cabre les dades de tots els canals, `0xE000` cau
**dins** la NVS en lloc d'en un espai buit, i cada `pio run -t upload` hi
escriu 8 KB de dades irrellevants a sobre.

**Impacte real, no confirmat**: si la NVS (basada en pàgines de 4 KB) tenia
dades emmagatzemades a les pàgines que cobreixen `0xE000`-`0xFFFF`, aquest
escriptura les hauria corromput silenciosament — sense cap error visible, ja
que aquest projecte no llegeix mai la partició `otadata` (no en té). No s'ha
investigat encara si el maquinari ja desplegat n'ha patit les conseqüències.

**Fix aplicat**: l'instal·lador `.bat` (`installer/` i `installer_standalone/`)
ja NO inclou `boot_app0.bin`. **Pendent**: el `pio run -t upload` de
PlatformIO (des de VS Code) segueix incloent-lo per defecte — no s'ha corregit
encara, cal fer-ho abans de tornar a flashejar des de l'IDE.

**Lliçó per als altres firmwares de la família**: qualsevol projecte que
ampliï la NVS més enllà del que la partició `otadata` per defecte, ocupa
(`0xE000`), ha de revisar què escriu realment `pio run -t upload` (amb `-v`),
no donar per fet que PlatformIO respecta la taula de particions personalitzada
en calcular què flashejar.

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

## Estat
Funcional i validat en maquinari real — BLE, cicle/escenes/transicions, DFPlayer i
persistència NVS operatius. Vegeu "Bugs trobats i corregits" per l'historial de problemes
reals detectats i corregits.
