# ProxiMicro - Kosmos Proxi Roboter mit micro:bit v2

## Projektübersicht

Dieses Projekt befasst sich mit der erweiterten Programmierung des Kosmos Proxi Roboters unter Verwendung des BBC micro:bit v2. Ziel ist die Nutzung von leistungsfähigen Programmiersprachen und RTOS-Funktionen für echtes Multithreading.

---

## MUSS-Regeln für Claude Code

### ⚠️ KRITISCH: Blockierende Befehle vermeiden (FREEZE-Gefahr!)

**Claude Code friert ein bei blockierenden Terminal-Befehlen!** Diese Befehle NIEMALS direkt ausführen:

#### Verbotene Befehle (FREEZE!)

| Kategorie | Verbotene Befehle |
|-----------|-------------------|
| **Serielle Ports** | `cu`, `screen`, `minicom`, `picocom`, `cat /dev/cu.*`, `cat /dev/tty.*` |
| **Debugger** | `pyocd gdbserver` (ohne Background), `openocd` (ohne Background), `gdb` (interaktiv) |
| **Netzwerk** | `nc -l` (listen), `telnet`, `ssh` (interaktiv) |
| **Sonstige** | `read`, `less`, `more`, `vim`, `nano`, jeder interaktive Befehl |

#### Sichere Alternativen

**1. Langläufige Prozesse IMMER im Hintergrund mit `run_in_background=true`:**
```bash
# RICHTIG: Background-Prozess
pyocd gdbserver -t nrf52833 &

# FALSCH: Blockiert!
pyocd gdbserver -t nrf52833
```

**2. Für Serial/Debug: tmux Session verwenden:**
```bash
# tmux installieren (einmalig)
brew install tmux

# Session erstellen und Befehl starten
tmux new-session -d -s debug 'pyocd gdbserver -t nrf52833'

# Status prüfen
tmux list-sessions

# Output lesen (nicht-blockierend)
tmux capture-pane -t debug -p

# Session beenden
tmux kill-session -t debug
```

**3. Output in Datei umleiten:**
```bash
# Prozess im Hintergrund mit Output-Datei
pyocd gdbserver -t nrf52833 > /tmp/pyocd.log 2>&1 &

# Log lesen (mit Limit!)
tail -20 /tmp/pyocd.log
```

**4. Python mit Timeout für serielle Ports (OHNE DTR/RTS Reset!):**
```python
import serial

# WICHTIG: DTR/RTS deaktivieren um Reset des micro:bit zu vermeiden!
ser = serial.Serial()
ser.port = '/dev/cu.usbmodemXXXX'
ser.baudrate = 115200
ser.timeout = 1      # timeout ist PFLICHT!
ser.dtr = False      # KEIN DTR - verhindert Reset!
ser.rts = False      # KEIN RTS - verhindert Reset!
ser.open()
```

**⚠️ WARNUNG:** Standard `serial.Serial('/dev/...')` toggelt DTR/RTS beim Öffnen und kann einen **Reset des micro:bit auslösen**, was BLE-Verbindungen unterbricht!

#### Checkliste vor Befehlsausführung

- [ ] Ist der Befehl interaktiv? → **NICHT ausführen**
- [ ] Wartet der Befehl auf Input? → **NICHT ausführen**
- [ ] Läuft der Befehl ewig? → **Background mit `&` oder `run_in_background=true`**
- [ ] Öffnet er einen Port/Socket? → **tmux oder Background**

---

### Serielle Ports

**NIEMALS** direkte Terminal-Tools wie `cu`, `screen`, `minicom` oder ähnliche blockierende Befehle verwenden, um serielle Ports zu öffnen. Diese blockieren das Terminal und können zum Crash führen.

**Stattdessen IMMER:**
1. **Python-Script mit pyserial** (mit Timeout):
```python
import serial
ser = serial.Serial('/dev/cu.usbmodemXXXX', 115200, timeout=1)
try:
    while True:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='ignore'), end='')
except KeyboardInterrupt:
    ser.close()
```

2. **Oder ein Tool im Hintergrund** mit Ausgabe in Datei:
```bash
# Beispiel: Output in Datei umleiten
stty -f /dev/cu.usbmodemXXXX 115200 raw -echo
cat /dev/cu.usbmodemXXXX > serial_output.log &
```

---

## Hardware

### micro:bit v2 Spezifikationen

| Parameter | Wert |
|-----------|------|
| **Main MCU** | Nordic nRF52833 |
| **Core** | ARM Cortex-M4F @ 64 MHz |
| **FPU** | Hardware Floating Point Unit |
| **Flash** | 512 KB |
| **RAM** | 128 KB |
| **Interface MCU** | nRF52833 oder nRF52820 (V2.2x) |
| **Bluetooth** | BLE 5.0 |

### Sensoren & Peripherie
- **Display**: 5x5 LED Matrix (auch als Lichtsensor nutzbar)
- **Buttons**: 2 programmierbare Tasten (A, B)
- **Accelerometer**: LSM303AGR (3-Achsen)
- **Magnetometer**: LSM303AGR (Kompass)
- **Mikrofon**: MEMS mit LED-Indikator
- **Lautsprecher**: Integriert
- **Touch**: Kapazitives Touch-Logo

### LSM303AGR Achsen-Zuordnung (Kalibriert 2026-01-24)

**Benutzer-Koordinatensystem (Referenzposition: USB oben, Display von dir weg):**
- **Z+** = USB-Stecker Richtung (nach oben)
- **X+** = von dir weg (Display schaut von dir weg)
- **Y+** = nach links (Richtung Button A)

```
        USB (Z+)
           ↑
   ┌───────────────┐
   │               │
Y+ │   Display     │ Y-
←  │   (5x5 LED)   │  →
   │   schaut      │
   │   nach X+     │
   └───────────────┘
     Edge Connector

   X+ = von dir weg (aus dem Display heraus)
```

**Gemessene Sensor-Rohwerte (milli-g bei 1g):**

| Position | Benutzer-Achse | Sensor-X | Sensor-Y | Sensor-Z |
|----------|----------------|----------|----------|----------|
| USB oben | Z+ oben | ~0 | **-1000** | ~0 |
| Button A oben | Y+ oben | **+1000** | ~0 | ~0 |
| USB unten | Z- oben | ~0 | **+1000** | ~0 |
| Display oben | X+ oben | ~0 | ~0 | **-1000** |

**Transformationsmatrix (Sensor → Benutzer):**

```
┌ User-X ┐   ┌  0  0 -1 ┐   ┌ Sensor-X ┐
│ User-Y │ = │ -1  0  0 │ × │ Sensor-Y │
└ User-Z ┘   └  0 -1  0 ┘   └ Sensor-Z ┘
```

**C-Code (in sensors.c):**
```c
float ax = -az_raw;   /* User-X = -Sensor-Z */
float ay = -ax_raw;   /* User-Y = -Sensor-X */
float az = -ay_raw;   /* User-Z = -Sensor-Y */

float mx = -mz_raw;   /* Magnetometer gleiche Transformation */
float my = -mx_raw;
float mz = -my_raw;
```

**Verifikation Roll/Pitch:**
| Bewegung | Erwartung | Gemessen |
|----------|-----------|----------|
| Nach rechts kippen (Btn B runter) | Roll + | ✓ Roll +44° |
| Nach links kippen (Btn A runter) | Roll - | ✓ Roll -30° |
| Nach vorne kippen (Display zum Boden) | Pitch + | ✓ Pitch +33° |
| Nach hinten kippen (Display zur Decke) | Pitch - | ✓ Pitch -45° |

### Stromversorgung
- USB-C (5V)
- Batterie-Connector (2x AAA)
- 3V Pad am Edge Connector
- Onboard Regulator: 300mA (190mA für Accessories verfügbar)

### Hardware Revisionen

| Version | Interface MCU | Hinweise |
|---------|---------------|----------|
| V2.00 | Freescale KL27 | Erste V2 Version |
| V2.2x | nRF52833 | Selber Chip wie Main MCU |
| V2.21 | nRF52820 | Alternative wegen Chipverfügbarkeit |

### Offizielle Hardware-Dokumentation

- **Schematic Repository**: https://github.com/microbit-foundation/microbit-v2-hardware
- **V2.0.0 Schematic PDF**: https://github.com/microbit-foundation/microbit-v2-hardware/blob/main/V2.00/MicroBit_V2.0.0_S_schematic.PDF
- **V2.2.1 Schematic PDF**: https://github.com/microbit-foundation/microbit-v2-hardware/blob/main/V2.21/MicroBit_V2.2.1_nRF52820%20schematic.PDF
- **Tech Site Hardware**: https://tech.microbit.org/hardware/
- **Schematic Docs**: https://tech.microbit.org/hardware/schematic/

### Edge Connector Pinout

| Pin | Funktion | nRF52833 GPIO |
|-----|----------|---------------|
| 0 | Large Pad / Analog | P0.02 |
| 1 | Large Pad / Analog | P0.03 |
| 2 | Large Pad / Analog | P0.04 |
| 3 | LED Col 1 | P0.31 |
| 4 | LED Col 2 | P0.28 |
| 5 | Button A | P0.14 |
| 6 | LED Col 9 | P1.05 |
| 7 | LED Col 8 | P0.11 |
| 8 | GPIO | P0.10 |
| 9 | LED Col 7 | P0.09 |
| 10 | LED Col 3 | P0.30 |
| 11 | Button B | P0.23 |
| 12 | GPIO | P0.12 |
| 13 | SCK (SPI) | P0.17 |
| 14 | MISO (SPI) | P0.01 |
| 15 | MOSI (SPI) | P0.13 |
| 16 | GPIO | P1.02 |
| 19 | I2C SCL | P0.26 |
| 20 | I2C SDA | P0.16 |

**Interne Pins:**
- UART TX (zum Interface): P0.06
- UART RX (vom Interface): P1.08
- Speaker: P0.00
- Microphone: P0.05 (Analog)
- Touch Logo: P1.04

### Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     micro:bit v2                             │
│                                                              │
│  ┌─────────────┐         ┌─────────────────────────────┐    │
│  │  Interface  │  UART   │      Main MCU               │    │
│  │    MCU      │◄───────►│      nRF52833               │    │
│  │ nRF52833/   │  SWD    │                             │    │
│  │ nRF52820    │◄───────►│  ┌─────┐  ┌────────────┐   │    │
│  │             │         │  │ BLE │  │ Cortex-M4F │   │    │
│  │  DAPLink    │         │  └─────┘  └────────────┘   │    │
│  └──────┬──────┘         │                             │    │
│         │                │  ┌─────────────────────┐    │    │
│         │ USB            │  │ 5x5 LED Matrix      │    │    │
│  ┌──────▼──────┐         │  └─────────────────────┘    │    │
│  │   USB-C     │         │                             │    │
│  │  Connector  │         │  ┌─────┐  ┌─────┐          │    │
│  └─────────────┘         │  │Btn A│  │Btn B│          │    │
│                          │  └─────┘  └─────┘          │    │
│  ┌─────────────┐         │                             │    │
│  │  I2C Bus    │◄───────►│  ┌──────────────────────┐  │    │
│  │ LSM303AGR   │         │  │ Edge Connector       │  │    │
│  │ Accel/Mag   │         │  │ (GPIO, I2C, SPI,     │  │    │
│  └─────────────┘         │  │  Analog, Power)      │  │    │
│                          │  └──────────────────────┘  │    │
│  ┌─────┐ ┌─────┐        └─────────────────────────────┘    │
│  │ Mic │ │ Spk │                                           │
│  └─────┘ └─────┘                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## Kosmos Proxi Roboter

### Übersicht
- **Artikelnummer**: 620585
- **Hersteller**: Franckh-Kosmos Verlags-GmbH & Co. KG
- **Produktseite**: https://www.kosmos.de/de/proxi_1620585_4002051620585
- **Anleitung**: https://www.manualslib.de/manual/1525570/Kosmos-Proxi.html
- **Support**: service@kosmos.de

### Proxi Zusatzplatine

| Komponente | Funktion |
|------------|----------|
| Edge Connector | Aufnahme für micro:bit |
| Infrarot-Sensoren | Hinderniserkennung, Linienfolgen |
| Buzzer | Akustische Ausgabe |
| Batteriefach-Anschluss | Stromversorgung (4x AA) |
| Motor-Treiber | Steuerung der Antriebsmotoren |

### Pin-Zuordnung (zu verifizieren durch Reverse Engineering)

| micro:bit Pin | Proxi Funktion (vermutet) |
|---------------|---------------------------|
| P0 | Motor Links PWM |
| P1 | Motor Rechts PWM |
| P2 | Buzzer |
| P8 | IR Sensor Links |
| P12 | IR Sensor Rechts |

**Hinweis**: Kein öffentlicher Schaltplan für die Proxi-Platine verfügbar!

---

## SDK/RTOS Optionen

### Option 1: Zephyr RTOS (Empfohlen)

Zephyr ist ein skalierbares, sicheres Echtzeit-Betriebssystem mit **offiziellem micro:bit v2 Support**.

**Vorteile:**
- Echte preemptive Threads
- Vollständige RTOS-Features (Semaphoren, Mutexe, Message Queues)
- Aktive Community und Dokumentation
- Board Definition `bbc_microbit_v2` vorhanden

**Dokumentation:**
- https://docs.zephyrproject.org/latest/boards/bbc/microbit_v2/doc/index.html
- https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/bbc/microbit_v2/
- https://zephyrproject.org/the-bbc-microbit-v2-and-openthread/

#### Zephyr Installation

```bash
# West (Zephyr Meta-Tool) installieren
pip install west

# Zephyr Workspace erstellen
west init ~/zephyrproject
cd ~/zephyrproject
west update

# Zephyr SDK installieren (von zephyrproject.org)
# macOS:
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_macos-x86_64.tar.xz
tar xf zephyr-sdk-0.16.8_macos-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh

# Python Dependencies
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt

# Environment
source ~/zephyrproject/zephyr/zephyr-env.sh
```

#### Zephyr Build & Flash

```bash
cd ~/zephyrproject/zephyr

# Build für micro:bit v2
west build -b bbc_microbit_v2 samples/basic/blinky

# Flashen (nutzt DAPLink automatisch)
west flash

# Clean Build
west build -t pristine
```

#### Zephyr Threading Beispiel

```c
#include <zephyr/kernel.h>

#define STACK_SIZE 1024
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(thread1_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread2_stack, STACK_SIZE);

struct k_thread thread1_data;
struct k_thread thread2_data;

void thread1_entry(void *p1, void *p2, void *p3)
{
    while (1) {
        printk("Thread 1 running\n");
        k_msleep(1000);
    }
}

void thread2_entry(void *p1, void *p2, void *p3)
{
    while (1) {
        printk("Thread 2 running\n");
        k_msleep(500);
    }
}

int main(void)
{
    k_thread_create(&thread1_data, thread1_stack,
                    K_THREAD_STACK_SIZEOF(thread1_stack),
                    thread1_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&thread2_data, thread2_stack,
                    K_THREAD_STACK_SIZEOF(thread2_stack),
                    thread2_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY + 1, 0, K_NO_WAIT);

    return 0;
}
```

#### Zephyr Synchronisation

```c
// Semaphore
K_SEM_DEFINE(my_sem, 0, 1);

// Mutex
K_MUTEX_DEFINE(my_mutex);

// Message Queue
K_MSGQ_DEFINE(my_msgq, sizeof(int), 10, 4);
```

#### Zephyr prj.conf für micro:bit v2

```conf
# Threading
CONFIG_MULTITHREADING=y
CONFIG_NUM_PREEMPT_PRIORITIES=16

# Bluetooth
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y

# Display
CONFIG_DISPLAY=y
CONFIG_MICROBIT_DISPLAY=y

# I2C Sensoren
CONFIG_I2C=y
CONFIG_SENSOR=y

# USB Console
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_CDC_ACM=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

# Debug
CONFIG_DEBUG=y
CONFIG_DEBUG_THREAD_INFO=y
```

---

### Option 2: CODAL (Lancaster University)

CODAL ist das offizielle Runtime der micro:bit Foundation mit Fiber-basiertem kooperativem Multitasking.

**Dokumentation:**
- https://tech.microbit.org/software/runtime/
- https://github.com/lancaster-university/codal-microbit-v2
- https://github.com/lancaster-university/microbit-v2-samples
- https://lancaster-university.github.io/microbit-docs/

#### CODAL Installation

```bash
# Voraussetzungen (macOS)
brew install cmake ninja arm-none-eabi-gcc python3

# Repository klonen
git clone https://github.com/lancaster-university/microbit-v2-samples
cd microbit-v2-samples

# Build
python build.py

# Output: MICROBIT.hex in build/
```

#### CODAL Fiber Beispiel

```cpp
#include "MicroBit.h"

MicroBit uBit;

void task1()
{
    while(true)
    {
        uBit.display.print("1");
        uBit.sleep(1000);  // Gibt Control ab
    }
}

void task2()
{
    while(true)
    {
        uBit.display.print("2");
        uBit.sleep(500);
    }
}

int main()
{
    uBit.init();

    create_fiber(task1);
    create_fiber(task2);

    while(true) {
        uBit.sleep(10000);
    }
}
```

#### Vergleich CODAL vs Zephyr

| Feature | CODAL | Zephyr |
|---------|-------|--------|
| Scheduling | Kooperative Fibers | Preemptive Threads |
| Komplexität | Einfacher | Mehr Features |
| micro:bit API | Nativ | Generic |
| RTOS Features | Basis | Vollständig |
| Footprint | Kleiner | Größer |

---

### Option 3: FreeRTOS (Experimentell)

FreeRTOS ist **nicht offiziell** für micro:bit v2 unterstützt, aber technisch möglich da der nRF52833 von FreeRTOS unterstützt wird.

**Empfehlung**: Zephyr bevorzugen, da es offiziellen Board-Support hat.

**Referenzen:**
- https://devzone.nordicsemi.com/f/nordic-q-a/75825/support-for-freertos-in-nrf52833
- https://forum.makecode.com/t/compile-c-and-freertos-for-microbit-v-2/17233

---

## Debugging ohne externen SWD-Debugger

Der micro:bit v2 hat einen **eingebauten CMSIS-DAP Debug-Adapter** auf dem Interface-Chip. Kein J-Link oder ST-Link erforderlich!

### Architektur

```
┌──────────┐      USB      ┌──────────────┐      SWD      ┌──────────┐
│   PC     │◄─────────────►│ Interface MCU │◄─────────────►│ Main MCU │
│          │               │ (DAPLink)     │               │ nRF52833 │
│ GDB/IDE  │               │ CMSIS-DAP     │               │ (Target) │
└──────────┘               └──────────────┘               └──────────┘
```

### Option 1: pyOCD (Empfohlen)

```bash
# Installation
pip install pyocd

# GDB Server starten
pyocd gdbserver -t nrf52833

# Mit Optionen
pyocd gdbserver -t nrf52833 --persist -f 1000000

# Firmware flashen
pyocd flash -t nrf52833 firmware.hex

# Mit Chip Erase
pyocd flash -t nrf52833 --erase chip firmware.hex
```

### Option 2: OpenOCD

```bash
# Installation (macOS)
brew install openocd

# GDB Server starten
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg

# Flashen
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg \
        -c "program firmware.hex verify reset exit"
```

### GDB Verbindung

```bash
# GDB starten
arm-none-eabi-gdb firmware.elf

# In GDB:
(gdb) target remote localhost:3333
(gdb) load
(gdb) monitor reset
(gdb) break main
(gdb) continue
```

### VS Code Integration

**Extensions:** `marus25.cortex-debug`

**launch.json:**
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug micro:bit (pyOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "pyocd",
            "cwd": "${workspaceRoot}",
            "executable": "${workspaceRoot}/build/firmware.elf",
            "targetId": "nrf52833",
            "runToEntryPoint": "main"
        }
    ]
}
```

### Nützliche GDB Befehle

| Befehl | Beschreibung |
|--------|--------------|
| `b main` | Breakpoint bei main |
| `c` | Continue |
| `n` | Next (Step Over) |
| `s` | Step Into |
| `p variable` | Variable ausgeben |
| `x/10x 0x20000000` | Memory dump |
| `info reg` | Register anzeigen |
| `bt` | Backtrace |
| `mon reset` | Target Reset |
| `mon halt` | Target anhalten |

**Dokumentation:**
- https://tech.microbit.org/software/daplink-interface/
- https://microbit-v2-debugging.readthedocs.io/
- https://pyocd.io/

---

## Bootloader & Flashen

### Standard: DAPLink

Der micro:bit verwendet DAPLink als Interface-Firmware:
- **Drag & Drop**: `.hex` Datei auf `MICROBIT` Laufwerk kopieren
- **CMSIS-DAP**: Debugging über USB
- **CDC Serial**: Serielle Konsole

**Dokumentation:** https://tech.microbit.org/software/daplink-interface/

### DAPLink Firmware Update

**Maintenance Mode betreten:**
1. RESET-Taste gedrückt halten
2. USB-Kabel anschließen
3. Laufwerk erscheint als `MAINTENANCE`
4. Neue DAPLink `.hex` Firmware kopieren

**Releases:** https://github.com/ARMmbed/DAPLink/releases

### Kommandozeilen-Flashen

**Detaillierte Anleitung:** [docs/bootloader/pyocd-flashing.md](docs/bootloader/pyocd-flashing.md)

#### Mit MCUboot (Empfohlen für OTA-fähige Firmware)

```bash
# 1. Chip löschen
pyocd erase -t nrf52833 --chip

# 2. MCUboot Bootloader flashen
pyocd flash -t nrf52833 -f 1000000 build/mcuboot/zephyr/zephyr.hex

# 3. Signierte Anwendung flashen
pyocd flash -t nrf52833 -f 1000000 build/firmware/zephyr/zephyr.signed.hex

# 4. Reset
pyocd reset -t nrf52833
```

#### Build-Script (imu_orientation)

```bash
cd imu_orientation/firmware
./build_flash.sh          # Nur bauen
./build_flash.sh flash    # Bauen + Flashen (MCUboot + App)
./build_flash.sh flash-app # Nur App flashen (MCUboot vorhanden)
./build_flash.sh ble      # OTA Update via Bluetooth
```

#### Ohne MCUboot (Einfach)

```bash
# pyOCD
pyocd flash -t nrf52833 firmware.hex

# OpenOCD
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg \
        -c "program firmware.hex verify reset exit"

# nrfjprog (Nordic Tool)
nrfjprog --program firmware.hex -f nrf52 --verify
nrfjprog --reset -f nrf52

# TinyGo
tinygo flash -target=microbit-v2 main.go
```

### Alternative: UF2 Bootloader

UF2 ist ein benutzerfreundliches Bootloader-Format (Doppelklick auf RESET aktiviert Boot-Modus).

```bash
# UF2 Datei erstellen
python uf2conv.py firmware.hex --family 0xADA52840 -o firmware.uf2

# UF2 Bootloader Repository
git clone https://github.com/makerdiary/uf2-bootloader
```

**Hinweis:** UF2-Installation ersetzt den originalen DAPLink!

### Bootloader Recovery

Falls Bootloader beschädigt:
1. **Maintenance Mode** versuchen (RESET beim USB-Anstecken)
2. Falls nicht möglich: **Externer SWD-Debugger** (J-Link, ST-Link) über SWD-Pads auf Rückseite

---

## Projekt-Struktur

```
ProxiMicro/
├── CLAUDE.md                           # Diese Datei
├── imu_orientation/                    # IMU Orientierungs-Firmware
│   └── firmware/
│       ├── src/                        # Quellcode
│       │   ├── main.c                  # Hauptprogramm
│       │   ├── sensors.c               # LSM303AGR Treiber
│       │   ├── orientation.c           # Orientierungsberechnung
│       │   ├── mahony_filter.c         # Mahony AHRS Filter
│       │   ├── hid_parser.c            # Xbox Controller HID Parser
│       │   ├── ble_central.c           # BLE Central (Controller)
│       │   ├── serial_output.c         # USB Serial Output
│       │   ├── ble_output.c            # BLE NUS Output
│       │   └── smp_bt.c                # BLE SMP Transport
│       ├── prj.conf                    # Zephyr Konfiguration
│       ├── CMakeLists.txt              # Build-Konfiguration
│       └── build_flash.sh              # Build & Flash Script
└── docs/
    ├── hardware/
    │   ├── microbit-v2-hardware.md     # Detaillierte Hardware-Docs
    │   ├── kosmos-proxi.md             # Proxi-spezifische Infos
    │   └── xbox-controller-ble-hid.md  # Xbox Controller BLE Protokoll
    ├── software/
    │   ├── zephyr-rtos.md              # Zephyr Setup & Beispiele
    │   ├── codal-sdk.md                # CODAL SDK Details
    │   └── freertos-option.md          # FreeRTOS Alternative
    ├── debugging/
    │   └── debug-setup.md              # pyOCD/OpenOCD Setup
    └── bootloader/
        ├── daplink-bootloader.md       # Bootloader Details
        └── pyocd-flashing.md           # pyOCD Programmier-Anleitung
```

---

## IMU Orientierungs-Firmware

### Übersicht

Die aktuelle Firmware (`imu_orientation/firmware/`) implementiert:
- **Echtzeit-Orientierung** mit LSM303AGR (Accelerometer + Magnetometer)
- **Mahony AHRS Filter** für stabile Roll/Pitch/Heading Berechnung
- **BLE Streaming** via Nordic UART Service (NUS)
- **Xbox Wireless Controller** Unterstützung via BLE HID
- **Pulsierende Herz-Animation** auf dem 5x5 LED Display im Idle-Zustand

### Xbox Controller Integration

Der micro:bit agiert als BLE Central und verbindet sich mit einem Xbox Wireless Controller.

**Detaillierte Protokoll-Dokumentation:** [docs/hardware/xbox-controller-ble-hid.md](docs/hardware/xbox-controller-ble-hid.md)

| Feature | Status |
|---------|--------|
| Analog Sticks (L/R) | ✓ 16-bit, Deadzone implementiert |
| Trigger (LT/RT) | ✓ 10-bit analog |
| D-Pad | ✓ 8-Wege Hat Switch |
| Face Buttons (A,B,X,Y) | ✓ |
| Bumpers (LB/RB) | ✓ |
| System (View/Menu/Xbox) | ✓ |
| Stick Clicks (L3/R3) | ✓ |

**Pairing:** Button A auf micro:bit drücken um Pairing-Modus zu starten.

### BLE Konfiguration

| Parameter | Wert |
|-----------|------|
| **Device Name** | `Ozzy` |
| **Service** | Nordic UART Service (NUS) |
| **TX UUID** | 6E400003-B5A3-F393-E0A9-E50E24DCCA9E |
| **RX UUID** | 6E400002-B5A3-F393-E0A9-E50E24DCCA9E |

### Datenformat (BLE/Serial)

```
IMU,<timestamp_ms>,<roll>,<pitch>,<heading>\r\n
```

Beispiel: `IMU,12345,2.5,-1.2,180.3\r\n`

### BLE Befehle

| Befehl | Antwort | Beschreibung |
|--------|---------|--------------|
| `CAL` | `CAL:OK` / `CAL:BUSY` | Magnetometer-Kalibrierung starten |
| `VER` | `VER:1.0.0-ble` | Firmware-Version abfragen |

### Build & Flash

```bash
cd imu_orientation/firmware

# Bauen (ohne MCUboot für direktes Flashen)
./build_flash.sh

# Bauen und Flashen
./build_flash.sh flash

# Nur Flashen (wenn bereits gebaut)
pyocd flash -t nrf52833 build/firmware/zephyr/zephyr.hex
```

### Aktuelle Konfiguration

MCUboot ist derzeit **deaktiviert** für einfaches direktes Flashen. Für OTA-Updates muss MCUboot in `prj.conf` wieder aktiviert werden.

---

## Quick Start

### 1. Zephyr installieren
```bash
pip install west
west init ~/zephyrproject && cd ~/zephyrproject && west update
```

### 2. Erstes Programm bauen
```bash
cd ~/zephyrproject/zephyr
west build -b bbc_microbit_v2 samples/basic/blinky
```

### 3. Flashen
```bash
west flash
```

### 4. Debuggen
```bash
pip install pyocd
pyocd gdbserver -t nrf52833
# In anderem Terminal:
arm-none-eabi-gdb build/zephyr/zephyr.elf -ex "target remote :3333"
```

---

## Wichtige Links

| Ressource | URL |
|-----------|-----|
| micro:bit Tech Site | https://tech.microbit.org/ |
| micro:bit v2 Hardware | https://github.com/microbit-foundation/microbit-v2-hardware |
| Zephyr micro:bit v2 | https://docs.zephyrproject.org/latest/boards/bbc/microbit_v2/doc/index.html |
| CODAL micro:bit v2 | https://github.com/lancaster-university/codal-microbit-v2 |
| pyOCD | https://pyocd.io/ |
| DAPLink | https://tech.microbit.org/software/daplink-interface/ |
| Kosmos Proxi | https://www.kosmos.de/de/proxi_1620585_4002051620585 |
