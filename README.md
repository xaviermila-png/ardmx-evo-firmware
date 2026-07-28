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

## Estat
En desenvolupament — encara no hi ha firmware funcional, només l'esquelet del projecte.
