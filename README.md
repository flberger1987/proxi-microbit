# ProxiMicro - Kosmos Proxi Hexapod Fernsteuerung

Zephyr RTOS Firmware für den BBC micro:bit v2 zur Steuerung des Kosmos Proxi Hexapod-Roboters via Xbox/PS5 Wireless Controller.

## Systemübersicht

```
┌─────────────────────┐      BLE Central      ┌─────────────────────┐
│  Xbox Wireless      │◄────────────────────►│    micro:bit v2      │
│  Controller         │      HID Reports      │    (nRF52833)        │
│  (oder PS5 DualSense)│                       │                      │
└─────────────────────┘                       │  ┌────────────────┐  │
                                              │  │ Zephyr RTOS    │  │
┌─────────────────────┐      BLE Peripheral   │  │ - Motor Thread │  │
│  Smartphone/PC      │◄────────────────────►│  │ - Sensor Thread│  │
│  (Debug Terminal)   │      Nordic UART      │  │ - Audio Thread │  │
└─────────────────────┘      Service (NUS)    │  │ - BLE Threads  │  │
                                              │  └────────────────┘  │
                                              └──────────┬───────────┘
                                                         │ PWM + GPIO
                                              ┌──────────▼───────────┐
                                              │  Kosmos Proxi        │
                                              │  Zusatzplatine       │
                                              │                      │
                                              │  ┌────────┐ ┌──────┐│
                                              │  │H-Bridge│ │Buzzer││
                                              │  │Motor 1 │ │      ││
                                              │  │Motor 2 │ └──────┘│
                                              │  └────────┘         │
                                              │      ▼              │
                                              │  ┌────────────────┐ │
                                              │  │ Hexapod        │ │
                                              │  │ 6x Beine       │ │
                                              │  └────────────────┘ │
                                              └─────────────────────┘
```

## Hardware-Komponenten

### BBC micro:bit v2

| Parameter | Wert |
|-----------|------|
| MCU | Nordic nRF52833 |
| Core | ARM Cortex-M4F @ 64 MHz |
| Flash | 512 KB |
| RAM | 128 KB |
| FPU | Hardware Floating Point |
| BLE | Bluetooth 5.0 |

**Integrierte Sensoren:**
- LSM303AGR Accelerometer + Magnetometer (I2C)
- MEMS Mikrofon
- Lautsprecher (PWM)
- 5x5 LED Matrix
- 2 programmierbare Buttons

### Kosmos Proxi Hexapod (Art.Nr. 620585)

Der Proxi ist ein **Hexapod** (6-Bein-Roboter) mit zwei Motoren:
- **Geh-Motor**: Vorwärts/Rückwärts-Bewegung aller Beine
- **Dreh-Motor**: Drehung um die eigene Achse (links/rechts)

**Stromversorgung:** 4x AA Batterien (6V)

---

## Pin-Belegung (Verifiziert)

### Motor-Steuerung (H-Brücke)

| micro:bit Pin | nRF GPIO | Funktion | PWM |
|---------------|----------|----------|-----|
| P13 | P0.17 | Geh-Motor: Vorwärts | PWM2 CH0 |
| P14 | P0.01 | Geh-Motor: Rückwärts | PWM2 CH1 |
| P15 | P0.13 | Dreh-Motor: Links | PWM3 CH0 |
| P16 | P1.02 | Dreh-Motor: Rechts | PWM3 CH1 |

### H-Brücken-Logik

**Geh-Motor (P13/P14):**
| P13 | P14 | Aktion |
|-----|-----|--------|
| HIGH/PWM | LOW | Vorwärts |
| LOW | HIGH/PWM | Rückwärts |
| LOW | LOW | Stopp |
| HIGH | HIGH | **VERBOTEN!** |

**Dreh-Motor (P15/P16):**
| P15 | P16 | Aktion |
|-----|-----|--------|
| HIGH/PWM | LOW | Links drehen |
| LOW | HIGH/PWM | Rechts drehen |
| LOW | LOW | Stopp |
| HIGH | HIGH | **VERBOTEN!** |

### Weitere Pins

| Pin | Funktion |
|-----|----------|
| P0.00 | Speaker (PWM1 CH0) |
| P8 (P0.10) | IR Sensor Links (ADC) |
| P12 (P0.12) | IR Sensor Rechts (ADC) |
| P19/P20 | I2C (LSM303AGR) |

---

## Controller-Mapping

### Xbox Wireless Controller (BLE HID)

| Controller | Roboter-Aktion |
|------------|----------------|
| **Linker Stick Y** | Vorwärts/Rückwärts |
| **Linker Stick X** | Links/Rechts drehen |
| **Rechter Trigger (RT)** | Gas (Geschwindigkeitsmodifikator) |
| **A-Button** | Sound: Schuss |
| **B-Button** | Sound: Klick |
| **X-Button** | Sound: Maschinengewehr |
| **Y-Button** | Sound: Hindernis-Warnung |

### PS5 DualSense (kompatibel)

Gleiches Mapping über BLE HID.

### HID Report Format (16 Bytes)

```
Byte  0-1:  Left Stick X   (uint16, center=32768)
Byte  2-3:  Left Stick Y   (uint16, center=32768, invertiert)
Byte  4-5:  Right Stick X  (uint16)
Byte  6-7:  Right Stick Y  (uint16)
Byte  8-9:  Left Trigger   (uint16, 10-bit: 0-1023)
Byte 10-11: Right Trigger  (uint16, 10-bit: 0-1023)
Byte 12:    D-Pad          (Hat Switch 0-8)
Byte 13:    Buttons Low    (A, B, X, Y, LB, RB)
Byte 14:    Buttons High   (View, Menu, Xbox, L3, R3)
Byte 15:    Reserved
```

**Deadzone:** 3277 (~10%) für Stick-Drift-Kompensation

---

## Software-Architektur

### Zephyr RTOS Thread-Struktur

```
┌─────────────────────────────────────────────────────────────────┐
│                        Zephyr Kernel                             │
├──────────────┬──────────────┬──────────────┬───────────────────┤
│ Motor Thread │ Sensor Thread│ Audio Thread │ BLE Central Thread│
│ Priority: 2  │ Priority: 5  │ Priority: 6  │ Priority: 3       │
│ 768B Stack   │ 1024B Stack  │ 512B Stack   │ 2048B Stack       │
│              │              │              │                   │
│ - PWM ctrl   │ - IMU read   │ - Tone gen   │ - Scan/Connect    │
│ - H-Bridge   │ - Orientation│ - Sound FX   │ - HID parsing     │
│ - 50Hz loop  │ - 20Hz loop  │ - Async play │ - Report handling │
├──────────────┴──────────────┴──────────────┼───────────────────┤
│ Main Thread                                 │ BLE Periph. Thread│
│ - Button handling                           │ - NUS Service     │
│ - Display animation                         │ - Debug output    │
│ - State machine                             │                   │
└─────────────────────────────────────────────┴───────────────────┘
```

### Quelldateien

| Datei | Beschreibung |
|-------|--------------|
| `main.c` | Hauptprogramm, State Machine, Display |
| `motor_driver.c` | PWM Motor-Steuerung, H-Brücken-Logik |
| `ble_central.c` | BLE Central für Controller-Verbindung |
| `hid_parser.c` | Xbox/PS5 HID Report Parser |
| `audio.c` | PWM Soundeffekte |
| `sensors.c` | LSM303AGR IMU Treiber |
| `orientation.c` | Roll/Pitch/Heading Berechnung |
| `mahony_filter.c` | Mahony AHRS Sensor-Fusion |
| `ble_output.c` | Nordic UART Service (Debug) |
| `smp_bt.c` | BLE Stack Initialisierung |
| `motor_test.c` | Serial-basierter Pin-Test |
| `robot_state.c` | Globaler Roboter-Zustand |
| `serial_output.c` | USB Serial Debug-Output |

### State Machine

```
                    ┌─────────┐
                    │  IDLE   │◄────────────┐
                    │ (Herz)  │             │
                    └────┬────┘             │
                         │ Button A         │ Disconnect/
                         │ Long Press       │ Cancel
                         ▼                  │
                    ┌─────────┐             │
                    │ PAIRING │─────────────┤
                    │ (Scan)  │             │
                    └────┬────┘             │
                         │ Controller       │
                         │ Found            │
                         ▼                  │
                    ┌─────────┐             │
                    │CONNECTED│─────────────┘
                    │ (Augen) │
                    └─────────┘
```

---

## Bedienung

### micro:bit Tasten

| Taste | Kurz drücken | Lang drücken (1s) |
|-------|--------------|-------------------|
| **A** | Klick-Sound | Pairing starten/beenden |
| **B** | Klick-Sound | Emergency Stop |

### Display-Anzeigen

| Animation | Bedeutung |
|-----------|-----------|
| Pulsierendes Herz | Idle - Warte auf Pairing |
| Rotierender Punkt | Pairing-Modus aktiv |
| Blinkende Augen | Controller verbunden |
| Haken (✓) | Aktion erfolgreich |
| X-Kreuz | Fehler |

### Controller-Pairing

1. micro:bit einschalten (Herz-Animation)
2. Controller in Pairing-Modus versetzen:
   - Xbox: Xbox-Taste 3 Sekunden halten
   - PS5: Share + PS-Taste gleichzeitig
3. Button A auf micro:bit lang drücken
4. Warten bis Augen-Animation erscheint
5. Steuerung aktiv!

---

## Build & Flash

### Voraussetzungen

```bash
# Zephyr SDK installieren
pip install west
west init ~/zephyrproject && cd ~/zephyrproject && west update

# pyOCD für Flashen
pip install pyocd

# Environment setzen
export ZEPHYR_BASE=~/zephyrproject/zephyr
source $ZEPHYR_BASE/zephyr-env.sh
```

### Kompilieren

```bash
cd imu_orientation/firmware
./build_flash.sh
```

### Flashen (USB)

```bash
# Vollständig (Chip Erase + Flash)
./build_flash.sh flash

# Nur Applikation (MCUboot muss vorhanden sein)
./build_flash.sh flash-app
```

### Manuelles Flashen mit pyOCD

```bash
# 1. Chip löschen
pyocd erase -t nrf52833 --chip

# 2. Firmware flashen
pyocd flash -t nrf52833 build/firmware/zephyr/zephyr.hex

# 3. Reset
pyocd reset -t nrf52833
```

### Drag & Drop (Alternative)

1. micro:bit per USB verbinden
2. `MICROBIT` Laufwerk öffnen
3. `zephyr.hex` auf Laufwerk kopieren

---

## Debugging

### Serial Console

```bash
# macOS - Port finden
ls /dev/cu.usbmodem*

# Python Serial Monitor (empfohlen)
python3 -c "
import serial
ser = serial.Serial('/dev/cu.usbmodemXXXX', 115200, timeout=1)
ser.dtr = False  # Kein Reset!
ser.rts = False
while True:
    line = ser.readline()
    if line: print(line.decode('utf-8', errors='ignore'), end='')
"
```

### Motor-Test Modus

Über Serial-Konsole können einzelne Pins getestet werden:

```
HELP      - Befehle anzeigen
SCAN      - Alle Pins durchschalten (2s pro Pin)
13 1      - P13 HIGH setzen
14 0      - P14 LOW setzen
ALL 0     - Alle Pins LOW
STATUS    - Pin-Zustände anzeigen
```

### GDB Debugging

```bash
# Terminal 1: GDB Server
pyocd gdbserver -t nrf52833

# Terminal 2: GDB
arm-none-eabi-gdb build/firmware/zephyr/zephyr.elf
(gdb) target remote :3333
(gdb) load
(gdb) break main
(gdb) continue
```

---

## BLE-Kommunikation

### Device Name: `Ozzy`

### Services

| Service | UUID | Beschreibung |
|---------|------|--------------|
| Nordic UART (NUS) | 6E400001-B5A3-F393-E0A9-E50E24DCCA9E | Debug I/O |
| HID Service | 0x1812 | Controller Input |

### NUS Befehle

| Befehl | Antwort | Beschreibung |
|--------|---------|--------------|
| `CAL` | `CAL:OK` | Magnetometer kalibrieren |
| `VER` | `VER:1.0.0` | Firmware-Version |

### IMU Datenformat (via NUS/Serial)

```
IMU,<timestamp_ms>,<roll>,<pitch>,<heading>\r\n
```

Beispiel: `IMU,12345,2.5,-1.2,180.3`

---

## Projekt-Struktur

```
ProxiMicro/
├── README.md                    # Diese Datei
├── CLAUDE.md                    # Entwickler-Referenz
├── .gitignore
│
├── imu_orientation/
│   └── firmware/
│       ├── CMakeLists.txt       # Zephyr Build-Konfiguration
│       ├── prj.conf             # Zephyr Projekt-Konfiguration
│       ├── build_flash.sh       # Build & Flash Script
│       │
│       ├── boards/
│       │   └── bbc_microbit_v2.overlay  # Device Tree Overlay
│       │
│       └── src/
│           ├── main.c           # Hauptprogramm
│           ├── motor_driver.c   # Motor-Steuerung
│           ├── ble_central.c    # BLE Controller-Verbindung
│           ├── hid_parser.c     # HID Report Parser
│           ├── audio.c          # Soundeffekte
│           ├── sensors.c        # IMU Treiber
│           └── ...
│
└── docs/
    ├── hardware/
    │   ├── kosmos-proxi.md      # Proxi Pin-Belegung
    │   ├── microbit-v2-hardware.md
    │   └── xbox-controller-ble-hid.md
    │
    ├── software/
    │   ├── zephyr-rtos.md
    │   └── codal-sdk.md
    │
    ├── debugging/
    │   └── debug-setup.md
    │
    └── bootloader/
        ├── daplink-bootloader.md
        └── pyocd-flashing.md
```

---

## Konfiguration (prj.conf)

Wichtige Einstellungen:

```conf
# BLE Dual-Role (Central + Peripheral)
CONFIG_BT=y
CONFIG_BT_CENTRAL=y           # Für Controller
CONFIG_BT_PERIPHERAL=y        # Für NUS Debug
CONFIG_BT_DEVICE_NAME="Ozzy"
CONFIG_BT_MAX_CONN=2

# PWM für Motoren und Speaker
CONFIG_PWM=y

# IMU Sensoren
CONFIG_I2C=y
CONFIG_SENSOR=y

# Floating Point (für Orientierung)
CONFIG_FPU=y
CONFIG_CBPRINTF_FP_SUPPORT=y

# Display
CONFIG_DISPLAY=y
CONFIG_MICROBIT_DISPLAY=y
```

---

## Bekannte Einschränkungen

1. **BLE Range:** ~10m indoor (typisch für BLE)
2. **Controller-Latenz:** ~120ms (BLE HID Report Rate)
3. **IR-Sensoren:** Noch nicht implementiert
4. **OTA-Update:** MCUboot derzeit deaktiviert

---

## Referenzen

- [micro:bit Tech Site](https://tech.microbit.org/)
- [Zephyr micro:bit v2](https://docs.zephyrproject.org/latest/boards/bbc/microbit_v2/doc/index.html)
- [Kosmos Proxi Produktseite](https://www.kosmos.de/de/proxi_1620585_4002051620585)
- [pyOCD Dokumentation](https://pyocd.io/)
- [Nordic nRF52833 Datasheet](https://infocenter.nordicsemi.com/pdf/nRF52833_PS_v1.5.pdf)

---

## Lizenz

SPDX-License-Identifier: Apache-2.0

---

*Dokumentation Stand: 2025-01-25*
