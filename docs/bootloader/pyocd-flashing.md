# Firmware Flashen mit pyOCD

## Übersicht

pyOCD ist ein Python-basiertes Tool für die Programmierung und das Debugging von ARM Cortex-M Mikrocontrollern. Es nutzt den eingebauten CMSIS-DAP Adapter des micro:bit v2.

**Offizielle Dokumentation:** https://pyocd.io/

## Installation

```bash
# Basis-Installation
pip install pyocd

# Mit UV (empfohlen)
uv pip install pyocd

# Target Pack für nRF52833 installieren (optional, meist automatisch)
pyocd pack install nrf52833
```

### Voraussetzungen prüfen

```bash
# Verfügbare Targets anzeigen
pyocd list --targets | grep -i nrf

# Verbundene Probes anzeigen
pyocd list --probes
```

Erwartete Ausgabe bei angeschlossenem micro:bit:
```
  #   Probe/Board             Unique ID            Target
--------------------------------------------------------------------------------
  0   BBC micro:bit CMSIS-DAP 99039000...          n/a
```

## Grundlegende Flash-Befehle

### HEX-Datei flashen

```bash
# Standard: Flash mit automatischem Sektor-Erase
pyocd flash -t nrf52833 firmware.hex

# Kurzform
pyocd flash -t nrf52 firmware.hex
```

### BIN-Datei flashen

Bei BIN-Dateien muss die Zieladresse angegeben werden:

```bash
# Flash ab Standard-Adresse 0x0
pyocd flash -t nrf52833 -a 0x0 firmware.bin

# Flash an spezifische Adresse (z.B. nach Bootloader)
pyocd flash -t nrf52833 -a 0x27000 application.bin
```

### ELF-Datei flashen

```bash
pyocd flash -t nrf52833 firmware.elf
```

## Erase-Modi

### Sektor-Erase (Standard)

Löscht nur die Sektoren, die für das Flashen benötigt werden:

```bash
pyocd flash -t nrf52833 --erase sector firmware.hex
```

### Chip-Erase

Löscht den gesamten Flash-Speicher vor dem Programmieren:

```bash
pyocd flash -t nrf52833 --erase chip firmware.hex
```

**Wann Chip-Erase verwenden:**
- Nach fehlgeschlagenen Flash-Versuchen
- Bei beschädigtem Flash-Inhalt
- Bei Wechsel zwischen unterschiedlichen Firmware-Typen
- Bei Problemen mit dem Mass-Erase

### Kein Erase

Überspringt das Löschen (nur für partielle Updates):

```bash
pyocd flash -t nrf52833 --erase none firmware.hex
```

## Verifikation

### Nach dem Flashen verifizieren

```bash
pyocd flash -t nrf52833 --verify firmware.hex
```

### Nur verifizieren (ohne Flashen)

```bash
# Flash-Inhalt lesen und mit Datei vergleichen
pyocd commander -t nrf52833 -c "compare 0x0 firmware.hex" -c "exit"
```

## Standalone Erase-Befehle

### Gesamten Chip löschen

```bash
pyocd erase -t nrf52833 --chip
```

### Bestimmten Adressbereich löschen

```bash
# Bereich von 0x0 bis 0x10000 löschen
pyocd erase -t nrf52833 --sector 0x0 0x10000
```

### Mass Erase (auch geschützten Flash)

```bash
pyocd erase -t nrf52833 --mass
```

## Flash-Geschwindigkeit

### SWD-Frequenz anpassen

```bash
# Höhere Frequenz für schnelleres Flashen (bis zu 4 MHz)
pyocd flash -t nrf52833 -f 4000000 firmware.hex

# Niedrigere Frequenz bei Verbindungsproblemen
pyocd flash -t nrf52833 -f 500000 firmware.hex
```

## Reset nach dem Flashen

### Automatischer Reset (Standard)

```bash
pyocd flash -t nrf52833 firmware.hex
# Reset erfolgt automatisch nach erfolgreichem Flash
```

### Reset überspringen

```bash
pyocd flash -t nrf52833 --no-reset firmware.hex
```

### Manueller Reset

```bash
pyocd reset -t nrf52833
```

## Mehrere Geräte

### Bestimmtes Gerät auswählen

```bash
# Unique ID anzeigen
pyocd list --probes

# Mit Unique ID flashen
pyocd flash -t nrf52833 -u 99039000... firmware.hex
```

### Alle angeschlossenen Geräte flashen

```bash
#!/bin/bash
for probe in $(pyocd list --probes --format=json | jq -r '.[].unique_id'); do
    echo "Flashing $probe..."
    pyocd flash -t nrf52833 -u "$probe" firmware.hex
done
```

## MCUboot + Anwendung Flashen

Wenn MCUboot als Bootloader verwendet wird, müssen **zwei Images** in der richtigen Reihenfolge geflasht werden:

### Flashing-Reihenfolge

```bash
# 1. Chip vollständig löschen
pyocd erase -t nrf52833 --chip

# 2. MCUboot Bootloader flashen (Adresse 0x0)
pyocd flash -t nrf52833 -f 1000000 build/mcuboot/zephyr/zephyr.hex

# 3. Signierte Anwendung flashen (Adresse 0xC000 = MCUboot Slot 0)
pyocd flash -t nrf52833 -f 1000000 build/firmware/zephyr/zephyr.signed.hex

# 4. Gerät zurücksetzen
pyocd reset -t nrf52833
```

### Warum diese Reihenfolge?

| Schritt | Grund |
|---------|-------|
| Chip Erase | Stellt sicher, dass kein alter Code Konflikte verursacht |
| MCUboot zuerst | Bootloader muss an Adresse 0x0 liegen |
| Signierte App | MCUboot validiert die Signatur beim Boot |
| Reset | Startet MCUboot, der dann die App lädt |

### Nur Anwendung aktualisieren

Wenn MCUboot bereits geflasht ist:

```bash
# Nur die neue Anwendung flashen
pyocd flash -t nrf52833 -f 1000000 build/firmware/zephyr/zephyr.signed.hex
pyocd reset -t nrf52833
```

### Build-Script

Für automatisiertes Build & Flash:

```bash
# Im Firmware-Verzeichnis
./build_flash.sh          # Nur bauen
./build_flash.sh flash    # Bauen und flashen (MCUboot + App)
./build_flash.sh flash-app # Nur App flashen
./build_flash.sh ble      # OTA Update via Bluetooth
./build_flash.sh clean    # Build-Verzeichnis löschen
```

---

## nRF52833 Flash-Speicher Layout

| Bereich | Adresse | Größe | Beschreibung |
|---------|---------|-------|--------------|
| Flash Start | 0x00000000 | - | Beginn des Flash |
| Application | 0x00000000 | 512 KB | Anwendungsspeicher |
| Flash Ende | 0x00080000 | - | Ende des 512 KB Flash |
| UICR | 0x10001000 | 4 KB | User Information Config |

### UICR programmieren

```bash
# UICR-Bereich flashen
pyocd flash -t nrf52833 -a 0x10001000 uicr.bin
```

## Troubleshooting

### "No connected debug probes"

```bash
# USB-Verbindung prüfen
ls /dev/cu.usbmodem*

# Probe-Liste aktualisieren
pyocd list --probes
```

### "Target not halted" oder "Flash write failed"

```bash
# 1. Chip vollständig löschen
pyocd erase -t nrf52833 --chip

# 2. Mit niedrigerer Frequenz flashen
pyocd flash -t nrf52833 -f 1000000 firmware.hex
```

### "Flash verification failed"

```bash
# Mass Erase und erneut flashen
pyocd erase -t nrf52833 --mass
pyocd flash -t nrf52833 --verify firmware.hex
```

### "Could not find pack for target"

```bash
# Target Pack installieren
pyocd pack install nrf52833

# Alternativ: Built-in Target verwenden
pyocd flash -t nrf52 firmware.hex
```

### Timeout beim Flashen

```bash
# Längeren Timeout setzen (in Sekunden)
pyocd flash -t nrf52833 --timeout 60 firmware.hex
```

## Automatisierungs-Script

```bash
#!/bin/bash
# flash-microbit.sh - Automatisiertes Flashen mit Fehlerbehandlung

HEX_FILE="$1"
TARGET="nrf52833"

if [ -z "$HEX_FILE" ]; then
    echo "Usage: $0 <firmware.hex>"
    exit 1
fi

if [ ! -f "$HEX_FILE" ]; then
    echo "Error: File not found: $HEX_FILE"
    exit 1
fi

echo "Checking connection..."
if ! pyocd list --probes | grep -q "micro:bit"; then
    echo "Error: No micro:bit found. Check USB connection."
    exit 1
fi

echo "Erasing chip..."
pyocd erase -t "$TARGET" --chip || exit 1

echo "Flashing $HEX_FILE..."
pyocd flash -t "$TARGET" --verify "$HEX_FILE" || exit 1

echo "Flash successful!"
```

## Python API Beispiel

```python
#!/usr/bin/env python3
"""Programmatisches Flashen mit pyOCD"""

from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer

def flash_microbit(hex_file: str) -> bool:
    """Flash HEX file to micro:bit v2."""
    try:
        with ConnectHelper.session_with_chosen_probe(
            target_override='nrf52833'
        ) as session:
            board = session.board
            target = board.target

            # Chip löschen
            target.mass_erase()

            # Flashen
            FileProgrammer(session).program(hex_file)

            # Reset
            target.reset()

            print(f"Successfully flashed {hex_file}")
            return True

    except Exception as e:
        print(f"Flash failed: {e}")
        return False

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python flash.py <firmware.hex>")
        sys.exit(1)

    success = flash_microbit(sys.argv[1])
    sys.exit(0 if success else 1)
```

## Vergleich: pyOCD vs Alternativen

| Feature | pyOCD | OpenOCD | nrfjprog |
|---------|-------|---------|----------|
| Installation | `pip install` | System-Paket | Nordic-Download |
| Plattformen | Win/Mac/Linux | Win/Mac/Linux | Win/Mac/Linux |
| Python API | Ja | Nein | Nein |
| CMSIS-DAP | Ja | Ja | Nein (J-Link) |
| Geschwindigkeit | Mittel | Mittel | Schnell |
| micro:bit Support | Exzellent | Gut | Gut |

## Referenzen

| Ressource | URL |
|-----------|-----|
| pyOCD Dokumentation | https://pyocd.io/docs |
| pyOCD GitHub | https://github.com/pyocd/pyOCD |
| pyOCD Target Packs | https://pyocd.io/docs/packs |
| nRF52833 Datasheet | https://infocenter.nordicsemi.com/pdf/nRF52833_PS_v1.5.pdf |
