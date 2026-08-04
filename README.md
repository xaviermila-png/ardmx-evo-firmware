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

## Bugs trobats i corregits

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
