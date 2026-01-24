# DAPLink und Bootloader für micro:bit v2

## Übersicht

Der micro:bit verwendet DAPLink als Interface-Firmware auf dem separaten Interface-Chip. Dies ermöglicht:
- Drag & Drop Programmierung (USB Mass Storage)
- CMSIS-DAP Debugging
- Serielle Konsole (CDC)

## Offizielle Dokumentation

- **DAPLink Interface**: https://tech.microbit.org/software/daplink-interface/
- **DAPLink Releases**: https://github.com/ARMmbed/DAPLink/releases
- **UF2 Bootloader**: https://github.com/makerdiary/uf2-bootloader

## DAPLink Architektur

```
┌────────────────────────────────────────────────────────────┐
│                    Interface MCU                            │
│                  (nRF52833 oder KL27)                       │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ Mass Storage│  │  CMSIS-DAP  │  │   CDC Serial        │ │
│  │  (DAPLINK)  │  │  (Debug)    │  │   (Console)         │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│         │                │                   │              │
│         └────────────────┼───────────────────┘              │
│                          │                                  │
│                     USB Stack                               │
└──────────────────────────┼──────────────────────────────────┘
                           │
                         USB-C
```

## Standard Programmierablauf

1. micro:bit über USB verbinden
2. Erscheint als `MICROBIT` Laufwerk
3. `.hex` Datei auf Laufwerk kopieren
4. DAPLink flasht automatisch den Main MCU
5. Programm startet

## DAPLink Firmware Update

### Für V2.00 (KL27 Interface)

#### Maintenance Mode betreten

1. **RESET-Taste gedrückt halten** während USB anschließen
2. Laufwerk erscheint als `MAINTENANCE`
3. Neue DAPLink Firmware (`.hex`) auf Laufwerk kopieren

#### Vollständiges Update via blhost

Für tiefere Updates kann das NXP Bootloader Host Tool verwendet werden:

```bash
# 1. TP1 mit GND verbinden während USB anschließen
#    (aktiviert KL27 internen Bootloader)

# 2. Zuerst nRF52 Flash löschen (wichtig!)
# Kopiere erase-flash.hex auf MICROBIT Laufwerk

# 3. blhost verwenden (von NXP herunterladen)
blhost -u -- flash-erase-all
blhost -u -- write-memory 0x0 daplink_full.bin
```

### Für V2.2x (nRF52 Interface)

Die V2.2x Versionen verwenden denselben nRF52-Chip für Interface und können über Maintenance Mode aktualisiert werden.

## Alternative: UF2 Bootloader

UF2 (USB Flashing Format) ist ein alternatives Bootloader-Format, das besonders benutzerfreundlich ist.

### Vorteile von UF2

- Einfache Dateiformat-Konvertierung
- Schnelleres Flashen
- Bessere Fehlerbehandlung
- Weit verbreitet (Adafruit, Raspberry Pi Pico, etc.)

### UF2 für nRF52833

```bash
# UF2 Bootloader Repository
git clone https://github.com/makerdiary/uf2-bootloader

# Build für nRF52833
cd uf2-bootloader
make BOARD=pca10100  # nRF52833-DK, ähnlich zum micro:bit
```

### UF2 Datei erstellen

```bash
# Aus HEX-Datei
python uf2conv.py firmware.hex --family 0xADA52840 -o firmware.uf2

# Aus BIN-Datei (Adresse angeben!)
python uf2conv.py firmware.bin -b 0x27000 --family 0xADA52840 -o firmware.uf2
```

### UF2 Bootloader flashen

**Achtung**: Dies ersetzt den originalen DAPLink Bootloader!

```bash
# Über pyOCD
pyocd flash -t nrf52833 uf2_bootloader.hex

# Oder über bestehenden DAPLink
# (Kopiere HEX auf MICROBIT Laufwerk)
```

### UF2 Modus aktivieren

Nach Installation des UF2 Bootloaders:
- **Doppelklick auf RESET** innerhalb 500ms
- Laufwerk erscheint als `BOOT`
- `.uf2` Dateien kopieren

## Kommandozeilen-Flashen

### Mit pyOCD

```bash
# HEX flashen
pyocd flash -t nrf52833 firmware.hex

# Mit vollständigem Chip Erase
pyocd flash -t nrf52833 --erase chip firmware.hex

# Nur bestimmten Bereich
pyocd flash -t nrf52833 -a 0x27000 firmware.bin
```

### Mit nrfjprog (Nordic Tool)

```bash
# Installation
# macOS: brew install nordicsemi/nrf/nrf-command-line-tools

# Chip löschen
nrfjprog --eraseall -f nrf52

# Firmware flashen
nrfjprog --program firmware.hex -f nrf52

# Reset
nrfjprog --reset -f nrf52
```

### Mit OpenOCD

```bash
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg \
        -c "program firmware.hex verify reset exit"
```

## TinyGo Flashen

```bash
# Standard (nutzt DAPLink automatisch)
tinygo flash -target=microbit-v2 main.go

# Mit SoftDevice (via OpenOCD)
tinygo flash -target=microbit-s110v8 -programmer=cmsis-dap main.go
```

## Bootloader Recovery

Falls der Bootloader beschädigt ist:

### Option 1: Maintenance Mode (wenn noch funktioniert)

1. RESET gedrückt halten + USB anschließen
2. Auf `MAINTENANCE` neue Firmware kopieren

### Option 2: Externer SWD-Debugger

Falls Maintenance Mode nicht mehr funktioniert, wird ein externer Debugger benötigt:
- J-Link
- ST-Link
- Raspberry Pi + OpenOCD

Verbindung über SWD Pads auf der Rückseite des micro:bit.

## Referenzen

| Ressource | URL |
|-----------|-----|
| DAPLink GitHub | https://github.com/ARMmbed/DAPLink |
| micro:bit DAPLink | https://tech.microbit.org/software/daplink-interface/ |
| UF2 Spec | https://github.com/microsoft/uf2 |
| pyOCD | https://pyocd.io/ |
