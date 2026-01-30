# Firmware Flashen mit pyOCD

## Übersicht

pyOCD ist ein Python-basiertes Tool für die Programmierung und das Debugging von ARM Cortex-M Mikrocontrollern. Es nutzt den eingebauten CMSIS-DAP Adapter des micro:bit v2.

**Offizielle Dokumentation:** https://pyocd.io/

## Installation

```bash
pip install pyocd

# Target Pack für nRF52833 (optional, meist automatisch)
pyocd pack install nrf52833
```

### Verbindung prüfen

```bash
# Verbundene Probes anzeigen
pyocd list --probes
```

Erwartete Ausgabe:
```
  #   Probe/Board             Unique ID            Target
--------------------------------------------------------------------------------
  0   BBC micro:bit CMSIS-DAP 99039000...          n/a
```

---

## ProxiMicro Flash-Layout

Das Projekt verwendet ein **Boot-Pointer Dual-Slot System** (kein MCUboot):

```
┌─────────────────────┬────────────┬─────────┐
│ Mini-Bootloader     │ 0x00000    │ 8 KB    │  ← Prüft Boot Control, springt zu Slot
├─────────────────────┼────────────┼─────────┤
│ Slot A (Primary)    │ 0x02000    │ 244 KB  │  ← Firmware Slot A
├─────────────────────┼────────────┼─────────┤
│ Slot B (Secondary)  │ 0x3F000    │ 244 KB  │  ← Firmware Slot B
├─────────────────────┼────────────┼─────────┤
│ Boot Control Block  │ 0x7C000    │ 4 KB    │  ← Aktiver Slot, CRCs, Flags
├─────────────────────┼────────────┼─────────┤
│ OTA State           │ 0x7D000    │ 4 KB    │  ← Transfer-Progress
├─────────────────────┼────────────┼─────────┤
│ Settings (NVS)      │ 0x7E000    │ 8 KB    │  ← Kalibrierung, PID, BLE Bonds
└─────────────────────┴────────────┴─────────┘
```

---

## Empfohlenes Flash-Verfahren

### Option 1: Build-Script (Empfohlen)

```bash
cd imu_orientation/firmware

# Initial-Flash (Bootloader + App) - LÖSCHT Settings!
./build_flash.sh flash

# Nur App flashen (behält Settings)
./build_flash.sh flash-app

# OTA via BLE (behält alles)
./build_flash.sh ota
```

### Option 2: Manuell mit pyOCD

#### Vollständiger Flash (Initial)

```bash
# 1. Chip vollständig löschen
pyocd erase -t nrf52833 --chip

# 2. Bootloader flashen (0x0)
pyocd flash -t nrf52833 build/bootloader/zephyr.hex

# 3. App flashen (0x2000 = Slot A)
pyocd flash -t nrf52833 build/app/zephyr.hex

# 4. Reset
pyocd reset -t nrf52833
```

#### Nur App aktualisieren (Settings behalten)

```bash
# Nur Slot A überschreiben (0x2000 - 0x3F000)
pyocd flash -t nrf52833 --erase sector build/app/zephyr.hex
pyocd reset -t nrf52833
```

**Hinweis:** Bei Sektor-Erase bleiben Settings (0x7E000+) erhalten!

---

## Grundlegende Flash-Befehle

### HEX-Datei flashen

```bash
pyocd flash -t nrf52833 firmware.hex
```

### BIN-Datei flashen (mit Adresse)

```bash
# Flash ab Slot A (0x2000)
pyocd flash -t nrf52833 -a 0x2000 firmware.bin
```

### Erase-Modi

| Modus | Befehl | Effekt |
|-------|--------|--------|
| Sektor | `--erase sector` | Nur betroffene Sektoren (Default) |
| Chip | `--erase chip` | Kompletter Flash **inkl. Settings!** |
| Keiner | `--erase none` | Nichts löschen |

```bash
# Settings behalten:
pyocd flash -t nrf52833 --erase sector firmware.hex

# Alles löschen:
pyocd flash -t nrf52833 --erase chip firmware.hex
```

---

## Datensicherheit

| Methode | Bootloader | App | Settings |
|---------|------------|-----|----------|
| `./build_flash.sh flash` | ✅ Neu | ✅ Neu | ❌ GELÖSCHT |
| `./build_flash.sh flash-app` | Unverändert | ✅ Neu | ✅ Erhalten |
| `./build_flash.sh ota` | Unverändert | ✅ Neu | ✅ Erhalten |
| `pyocd --erase chip` | ❌ Gelöscht | ❌ Gelöscht | ❌ GELÖSCHT |
| `pyocd --erase sector` | Unverändert | ✅ Neu | ✅ Erhalten |

**Empfehlung:** Nach initialem `flash` immer `ota` oder `flash-app` verwenden!

---

## Verifikation

```bash
# Nach dem Flashen verifizieren
pyocd flash -t nrf52833 --verify firmware.hex
```

---

## Reset

```bash
# Manueller Reset
pyocd reset -t nrf52833

# Halt (für Debugging)
pyocd commander -t nrf52833 -c "halt"
```

---

## Troubleshooting

### "No connected debug probes"

```bash
ls /dev/cu.usbmodem*
pyocd list --probes
```

### "Flash write failed"

```bash
# Chip löschen und neu flashen
pyocd erase -t nrf52833 --chip
pyocd flash -t nrf52833 firmware.hex
```

### "Flash verification failed"

```bash
pyocd erase -t nrf52833 --mass
pyocd flash -t nrf52833 --verify firmware.hex
```

### Langsames Flashen

```bash
# Höhere SWD-Frequenz (bis 4 MHz)
pyocd flash -t nrf52833 -f 4000000 firmware.hex
```

---

## Referenzen

| Ressource | URL |
|-----------|-----|
| pyOCD Dokumentation | https://pyocd.io/docs |
| pyOCD GitHub | https://github.com/pyocd/pyOCD |
| nRF52833 Datasheet | https://infocenter.nordicsemi.com/pdf/nRF52833_PS_v1.5.pdf |
