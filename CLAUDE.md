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

### Pin-Zuordnung (Verifiziert 2025-01-25)

Der Kosmos Proxi ist ein **Hexapod** (6-Bein-Roboter), kein Radfahrzeug!

**Motor-Pins:**

| micro:bit Pin | GPIO | Proxi Funktion |
|---------------|------|----------------|
| P13 | P0.17 | Hexapod Vorwärts gehen |
| P14 | P0.01 | Hexapod Rückwärts gehen |
| P15 | P0.13 | Hexapod Links drehen |
| P16 | P1.02 | Hexapod Rechts drehen |

**IR-Sensor-Pins:**

| micro:bit Pin | GPIO | ADC | Proxi Funktion |
|---------------|------|-----|----------------|
| P0 | P0.02 | AIN0 | IR Sensor Links |
| P1 | P0.03 | AIN1 | IR Sensor Rechts |
| P12 | P0.12 | GPIO | IR LED Enable (Active-HIGH) |

**Motor-Konfiguration:**
- **Geh-Motor:** H-Brücke mit P13 (vorwärts) / P14 (rückwärts)
- **Dreh-Motor:** H-Brücke mit P15 (links) / P16 (rechts)
- Nur ein Pin pro Richtung HIGH, niemals beide gleichzeitig!

**IR-Sensor-Konfiguration:**
- IR-LEDs werden über P12 ein/ausgeschaltet
- Differenzmessung (LED on - LED off) eliminiert Fremdlicht
- ADC mit 4x Gain für maximale Empfindlichkeit

**Hinweis**: Reverse-Engineering durchgeführt mittels GPIO-Scan.

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
CONFIG_MAIN_STACK_SIZE=3072

# GPIO & I2C
CONFIG_GPIO=y
CONFIG_I2C=y
CONFIG_SENSOR=y

# Display
CONFIG_DISPLAY=y
CONFIG_MICROBIT_DISPLAY=y

# Bluetooth (Dual-Role: Central + Peripheral)
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_CENTRAL=y
CONFIG_BT_GATT_CLIENT=y
CONFIG_BT_MAX_CONN=2
CONFIG_BT_MAX_PAIRED=2
CONFIG_BT_SMP=y
CONFIG_BT_BONDABLE=y
CONFIG_BT_SETTINGS=y
CONFIG_BT_ZEPHYR_NUS=y

# Persistent Storage für Bonding
CONFIG_SETTINGS=y
CONFIG_NVS=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_FLASH_PAGE_LAYOUT=y

# PWM für Motoren und Speaker
CONFIG_PWM=y

# ADC für IR Sensoren
CONFIG_ADC=y

# FPU für Quaternion-Berechnungen
CONFIG_FPU=y
CONFIG_CBPRINTF_FP_SUPPORT=y

# Console
CONFIG_PRINTK=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

# Heap (für BLE)
CONFIG_HEAP_MEM_POOL_SIZE=8192
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
├── CLAUDE.md                           # Diese Datei (Developer Reference)
├── README.md                           # Projekt-Übersicht & Quick Start
├── controller_mapping.py               # Xbox Controller Mapping Reference
├── serial_monitor.py                   # Serial Port Monitor Tool
│
├── tools/
│   └── telemetry_receiver.py           # BLE Telemetry Empfänger (bleak)
│
├── imu_orientation/                    # IMU Orientierungs-Firmware
│   ├── firmware/
│   │   ├── src/                        # Quellcode (18 .c + 17 .h)
│   │   │   ├── main.c                  # Hauptprogramm, State Machine, Display
│   │   │   ├── robot_state.c/h         # Globaler Roboter-Zustand
│   │   │   ├── sensors.c/h             # LSM303AGR IMU Treiber
│   │   │   ├── orientation.c/h         # Roll/Pitch/Heading Berechnung
│   │   │   ├── mahony_filter.c/h       # Mahony AHRS Filter (Quaternion)
│   │   │   ├── ble_central.c/h         # BLE Central (Xbox Controller)
│   │   │   ├── hid_parser.c/h          # Xbox/PS5 HID Report Parser
│   │   │   ├── motor_driver.c/h        # PWM H-Brücke Steuerung
│   │   │   ├── autonomous_nav.c/h      # Autonome Navigation
│   │   │   ├── yaw_controller.c/h      # PID Yaw Rate Regelung
│   │   │   ├── ir_sensors.c/h          # IR Hindernissensoren (ADC+Kalman)
│   │   │   ├── audio.c/h               # PWM Tongenerierung
│   │   │   ├── telemetry.c/h           # Binary Telemetrie (~20Hz)
│   │   │   ├── ble_output.c/h          # Nordic UART Service (NUS)
│   │   │   ├── smp_bt.c/h              # BLE Stack Initialisierung
│   │   │   ├── serial_output.c/h       # USB Serial Debug Output
│   │   │   ├── motor_test.c/h          # GPIO Test Modus
│   │   │   └── sysid.c/h               # System Identification Test
│   │   │
│   │   ├── boards/
│   │   │   └── bbc_microbit_v2.overlay # Device Tree (PWM Pins)
│   │   │
│   │   ├── sysbuild/
│   │   │   └── mcuboot.conf            # MCUboot Bootloader Konfig
│   │   │
│   │   ├── tools/
│   │   │   ├── imu_plot.py             # Echtzeit IMU Plotting
│   │   │   └── ir_plot.py              # IR Sensor Plotting
│   │   │
│   │   ├── prj.conf                    # Zephyr Haupt-Konfiguration
│   │   ├── bt.conf                     # Alternative BLE Konfiguration
│   │   ├── sysbuild.conf               # MCUboot Sysbuild Konfig
│   │   ├── CMakeLists.txt              # Build-Konfiguration
│   │   └── build_flash.sh              # Build & Flash Script
│   │
│   └── visualizer/                     # 3D Orientierungs-Visualisierer
│       ├── visualizer.py               # Hauptprogramm (PyGame/OpenGL)
│       ├── cube_renderer.py            # PyOpenGL Cube Renderer
│       ├── serial_reader.py            # Serial Data Reader
│       └── requirements.txt            # Python Dependencies
│
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

**Sensorik:**
- **Echtzeit-Orientierung** mit LSM303AGR (Accelerometer + Magnetometer)
- **Mahony AHRS Filter** (Quaternion) für Roll/Pitch aus Accel+Mag
- **2D Kalman-Filter** für Heading + Yaw Rate (θ, ω) mit Motor-Feedforward
- **1D Kalman-Filter** für IR-Distanzschätzung (mm)
- **Magnetometer-Kalibrierung** (Hard-Iron Kompensation, Flash-persistent)

**Kommunikation:**
- **BLE Dual-Role**: Central (Xbox Controller) + Peripheral (NUS Debug)
- **Binary Telemetrie** ~20Hz über BLE NUS (36-Byte Pakete)
- **USB Serial Debug** Output (115200 Baud)
- **BLE Auto-Reconnect** mit persistentem Bonding (überlebt Neustart)

**Steuerung:**
- **Xbox Wireless Controller** via BLE HID
- **PS5 Controller** Unterstützung (experimentell)
- **Autonome Navigation** mit Heading-Hold und Hindernisvermeidung
- **PID Yaw Rate Control** für präzise Drehungen

**Aktuatoren:**
- **PWM Motor-Steuerung** für Hexapod (Gehen + Drehen)
- **PWM Audio** Tongenerierung mit Sound-Effekten
- **5x5 LED Display** Animationen (Herz, Augen, Pfeil, Emoji)

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

**Pairing:** Button A (long press 1s) auf micro:bit drücken um Pairing-Modus zu starten.

### BLE Auto-Reconnect (Implementiert 2025-01-25)

Nach erfolgreichem Pairing speichert der micro:bit die Bond-Informationen persistent im Flash. Die automatische Wiederverbindung funktioniert wie folgt:

**Nach Neustart des micro:bit:**
1. Bond-Info wird aus Flash geladen
2. Scan startet automatisch → drehender Punkt (30s)
3. Wenn Controller gefunden → automatische Verbindung
4. Wenn Timeout → Herz-Animation (10s Pause) → erneuter Scan

**Bei Verbindungsabbruch (Controller aus/Reichweite):**
1. Disconnect erkannt → Herz-Animation
2. Nach 2s: Scan startet → drehender Punkt
3. Wenn Controller wieder eingeschaltet → automatische Verbindung
4. Wenn Timeout → Herz (10s) → Retry

**Display-Zustände:**
| Animation | Bedeutung |
|-----------|-----------|
| ❤️ Pulsierendes Herz | Idle - wartet auf Controller |
| 🔄 Drehender Punkt | Scanning - sucht Controller |
| 👀 Blinkende Augen | Connected - Controller verbunden |
| ➡️ Pfeil | Autonomous Mode aktiv |

**Manuelles Eingreifen:**
- Long-press Button A im Idle-Zustand: Neuen Scan starten
- Long-press Button A während Scan: Scan abbrechen
- Long-press Button A bei Verbindung: Controller trennen

### Autonomer Navigationsmodus

Der Roboter kann autonom navigieren mit Heading-Hold und IR-basierter Hindernisvermeidung.

**Aktivierung:** D-Pad UP auf dem Xbox Controller

**D-Pad Steuerung (im Autonomous Mode):**

| D-Pad | Aktion | Beschreibung |
|-------|--------|--------------|
| **UP** | Aktivieren | Startet Autonomous Mode, kein Effekt wenn bereits aktiv |
| **DOWN** | 180° U-Turn | Dreht um und fährt zurück |
| **LEFT** | -90° Drehung | Dreht gegen Uhrzeigersinn |
| **RIGHT** | +90° Drehung | Dreht im Uhrzeigersinn |

**Wichtig:**
- D-Pad-Eingaben werden ignoriert während einer Drehung läuft
- D-Pad deaktiviert **niemals** den Autonomous Mode
- Nur **Stick/Trigger >10%** = Manual Override = Autonomous Mode beendet

**Zustände (enum autonav_state):**

| Zustand | Wert | Beschreibung |
|---------|------|--------------|
| `AUTONAV_DISABLED` | 0 | Navigation deaktiviert, manuelle Steuerung |
| `AUTONAV_YAW_TEST_CW` | 1 | Yaw-Test: Rotation im Uhrzeigersinn (5s) |
| `AUTONAV_YAW_TEST_CCW` | 2 | Yaw-Test: Rotation gegen Uhrzeigersinn (5s) |
| `AUTONAV_YAW_TEST_DONE` | 3 | Yaw-Test: Abgeschlossen, zeigt Ergebnisse |
| `AUTONAV_HEADING_HOLD` | 4 | Fährt vorwärts, hält Ziel-Heading, vermeidet Hindernisse |
| `AUTONAV_TURNING` | 5 | Dreht auf neues Ziel-Heading (proportionale Geschwindigkeit) |
| `AUTONAV_BACKING_UP` | 6 | Fährt rückwärts (beide IR < 150mm), dann Wegdrehen |

**Hindernisvermeidung (Kalman-gefiltert):**
- **> 350mm**: Volle Geschwindigkeit (100%), nur Heading-Korrektur
- **150-350mm**: Proportionale Vermeidung, reduzierte Geschwindigkeit
- **< 150mm** (beide Seiten): Rückwärtsfahren, dann intelligentes Wegdrehen

**Wegdrehverhalten nach Backup:**
- Proportionaler Winkel basierend auf IR-Differenz (45°-135°)
- Mehr Platz rechts → dreht nach rechts
- Beide Seiten gleich eng → 180° U-Turn

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
| `IMU` | `IMU:ON` / `IMU:OFF` | IMU Text-Streaming togglen |
| `TELE` | `TELE:ON` / `TELE:OFF` | Binary-Telemetrie togglen |
| `TELEON` | `TELE:ON` | Binary-Telemetrie aktivieren |
| `TELEOFF` | `TELE:OFF` | Binary-Telemetrie deaktivieren |
| `IRD` | `IRD:ON` / `IRD:OFF` | IR Debug-Ausgabe togglen |
| `HELP` | `CMD:VER,CAL,...` | Verfügbare Befehle auflisten |

### BLE Binary-Telemetrie (Dashboard)

Die Firmware sendet ~20Hz binary Telemetrie-Pakete über NUS.

**Verbindungsstrategie:** Dashboard **vor** Xbox Controller verbinden!
1. Dashboard via NUS verbinden
2. Dann Xbox Controller verbinden
3. NUS bleibt aktiv (Advertising pausiert nur)

**Paketformat (36 Bytes, Little-Endian):**

```c
struct telemetry_packet {
    uint8_t  magic;              /* 0xAB - Sync-Byte */
    uint8_t  version;            /* 0x01 - Protokoll-Version */
    uint32_t timestamp_ms;       /* Uptime in ms */

    int16_t  roll_x10;           /* Roll * 10 (0.1° Auflösung) */
    int16_t  pitch_x10;          /* Pitch * 10 */
    uint16_t heading_x10;        /* Heading * 10 (0-3599) */

    int16_t  yaw_rate_x10;       /* Yaw Rate * 10 (°/s) */
    uint16_t ir_left_mm;         /* IR links in mm */
    uint16_t ir_right_mm;        /* IR rechts in mm */

    int8_t   motor_linear;       /* -100 bis +100 */
    int8_t   motor_angular;      /* -100 bis +100 */
    uint8_t  nav_state;          /* 0=DISABLED, 1=HEADING_HOLD, 2=TURNING, 3=BACKING_UP */
    uint8_t  flags;              /* Bit 0: autonav, Bit 1: motors */

    uint16_t target_heading_x10; /* Ziel-Heading * 10 (0xFFFF = invalid) */

    int16_t  raw_ax, raw_ay, raw_az; /* Accelerometer (milli-g) */
    int16_t  raw_mx, raw_my, raw_mz; /* Magnetometer (milli-Gauss) */
};
```

**Python struct format:** `'<BBIhhHhHHbbBBHhhhhhh'` (36 Bytes)

| Offset | Format | Feld | Quelle |
|--------|--------|------|--------|
| 0 | B | magic | 0xAB Sync-Byte |
| 1 | B | version | 0x01 Protokoll |
| 2 | I | timestamp_ms | Uptime (k_uptime_get_32) |
| 6 | h | roll_x10 | Mahony Filter (Quaternion → Euler) |
| 8 | h | pitch_x10 | Mahony Filter (Quaternion → Euler) |
| 10 | H | heading_x10 | **2D Kalman Filter** (θ-Schätzung) |
| 12 | h | yaw_rate_x10 | **2D Kalman Filter** (ω-Schätzung) |
| 14 | H | ir_left_mm | **1D Kalman Filter** (Distanz) |
| 16 | H | ir_right_mm | **1D Kalman Filter** (Distanz) |
| 18 | b | motor_linear | Motor Command (-100 bis +100) |
| 19 | b | motor_angular | Motor Command (-100 bis +100) |
| 20 | B | nav_state | enum autonav_state (0-6) |
| 21 | B | flags | Bit 0: autonav, Bit 1: motors |
| 22 | H | target_heading_x10 | Autonomous Nav Target |
| 24 | hhh | raw_ax/ay/az | **Roh** Accel (milli-g, User coords) |
| 30 | hhh | raw_mx/my/mz | **Roh** Mag (milli-Gauss, User coords) |

**Kalman-Filter Details:**
- **Heading + Yaw Rate**: 2D Kalman mit Prozessmodell `θ' = θ + ω·dt`, `ω' = a·ω + b·motor_cmd`
- **IR Distanz**: 1D Kalman für Glättung der ADC→mm Konvertierung

**Test-Script:** `tools/telemetry_receiver.py` (benötigt `pip install bleak`)

### Sensorfusion & Kalman-Filter

**Orientierung (Roll/Pitch):**
- Mahony AHRS Filter (Quaternion-basiert)
- Fusioniert Accelerometer + Magnetometer
- Output: Roll, Pitch in Grad

**Heading + Yaw Rate (2D Kalman, `sensors.c`):**

Der 2D Kalman-Filter schätzt Heading (θ) und Yaw Rate (ω) gemeinsam:

```
Zustandsvektor: x = [θ, ω]^T

Prozessmodell:
  θ' = θ + ω·dt
  ω' = a·ω + b·motor_cmd    (a=0.95, b=0.4)

Messmodell:
  z = θ_magnetometer        (nur Heading wird gemessen)
```

**Parameter:**
- `Q_THETA = 0.1` (Prozessrauschen Heading)
- `Q_OMEGA = 1.0` (Prozessrauschen Yaw Rate)
- `R_THETA = 2.0` (Messrauschen Magnetometer)
- `DT = 0.1s` (10 Hz Update Rate)

**IR-Distanz (1D Kalman, `ir_sensors.c`):**

Einfacher 1D Kalman für Glättung der ADC→mm Konvertierung:

```
x' = x                      (konstantes Modell)
z = distance_mm             (gemessene Distanz)
```

**Parameter:**
- `Q = 1.0` (Prozessrauschen)
- `R = 10.0` (Messrauschen)
- Initial: P = 100.0, x = 550mm

### Magnetometer-Kalibrierung

Die Magnetometer-Kalibrierung kompensiert Hard-Iron-Störungen (Offset) durch Metall/Magnete in der Nähe des Sensors.

**Kalibrierung starten:**
1. **Long-press Button B** (3 Sekunden gedrückt halten)
2. **30 Sekunden** lang den micro:bit durch alle Orientierungen drehen
3. Dabei alle Achsen abdecken (wie eine Kugel abrollen)
4. Kalibrierung wird automatisch im Flash gespeichert

**Serielle Ausgabe während Kalibrierung:**
```
CAL,START
CAL,<sample>,<mx>,<my>,<mz>
...
CAL,DONE,<offset_x>,<offset_y>,<offset_z>
MAG CAL: Saved to flash
```

**Wichtig:**
- Kalibrierung wird im Flash persistent gespeichert (überlebt Neustart)
- **Beim Flashen mit `pyocd erase --chip` geht die Kalibrierung verloren!**
- Normales Flashen mit `pyocd flash` behält die Kalibrierung (Settings-Partition bleibt)
- Nach Verlust: Neu kalibrieren mit Long-press Button B

**Kalibrierung via BLE:**
```
# Senden: CAL
# Antwort: CAL:OK (Kalibrierung gestartet) oder CAL:BUSY (läuft bereits)
```

### IR-Sensoren

Die Proxi-Platine hat zwei IR-Reflexionssensoren für Hindernisvermeidung.

**Pin-Belegung:**

| Funktion | micro:bit Pin | nRF52833 | ADC |
|----------|---------------|----------|-----|
| IR Links | P0 | P0.02 | AIN0 |
| IR Rechts | P1 | P0.03 | AIN1 |
| IR LED Enable | P12 | P0.12 | GPIO (Active-HIGH) |

**Messprinzip:**
1. ADC-Messung mit IR-LEDs **aus** (Ambient-Licht)
2. IR-LEDs einschalten (P12=HIGH)
3. ADC-Messung mit IR-LEDs **an** (Reflection)
4. Differenz = tatsächlicher IR-Wert (eliminiert Fremdlicht)

**Signalverarbeitung:**
1. Differenzmessung (IR on - IR off) → Roher ADC-Wert
2. Tiefpass-Filter (Alpha=0.3) für Rauschunterdrückung
3. Bias-Kompensation für Ambient-Licht
4. ADC → mm Konvertierung (Lookup-Tabelle)
5. **1D Kalman-Filter** für finale Distanzschätzung (Q=1.0, R=10.0)

**Distanz-Mapping (kalibriert):**

| ADC-Wert | Distanz |
|----------|---------|
| 0-50 | > 400mm (kein Hindernis) |
| 50-200 | 400mm - 150mm |
| 200-400 | 150mm - 50mm |
| > 400 | < 50mm (sehr nah) |

**API (ir_sensors.h):**
```c
void ir_sensors_get_distance(float *left_mm, float *right_mm);
void ir_sensors_get_raw(uint16_t *left, uint16_t *right);
void ir_sensors_set_debug(bool enabled);
```

### Audio-Modul

PWM-basierte Tongenerierung auf dem integrierten Lautsprecher (P0.00).

**Sound Events (enum sound_event):**

| Event | Beschreibung |
|-------|--------------|
| `SOUND_PAIRING_START` | Aufsteigender Ton (Scan startet) |
| `SOUND_CONNECTED` | Erfolgsmelodie (Controller verbunden) |
| `SOUND_DISCONNECTED` | Absteigender Ton (Verbindung verloren) |
| `SOUND_OBSTACLE` | Warnton (Hindernis) |
| `SOUND_BUTTON_PRESS` | Kurzer Klick |
| `SOUND_ERROR` | Fehlerton |
| `SOUND_SHOOT` | Laser/Schuss-Effekt (Button A) |
| `SOUND_MACHINEGUN` | Maschinengewehr (Button B gehalten) |
| `SOUND_CALIBRATION_BEEP` | Kurzer Beep während Kalibrierung |
| `SOUND_CALIBRATION_DONE` | Erfolgs-Jingle nach Kalibrierung |

**Proximity Beeper:**
- Parksensor-ähnliche Funktion
- Beep-Rate proportional zur Nähe
- Aktiviert sich automatisch bei IR < 300mm

**Noten-Frequenzen (Hz):**
```c
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
...
#define NOTE_C5  523
```

### Motor-Treiber

PWM-basierte H-Brücken-Steuerung für den Hexapod.

**PWM-Zuordnung:**

| PWM | Funktion | Pins |
|-----|----------|------|
| PWM1 | Speaker | P0.00 |
| PWM2 | Geh-Motor | P13 (fwd), P14 (bwd) |
| PWM3 | Dreh-Motor | P15 (links), P16 (rechts) |

**API (motor_driver.h):**
```c
void motor_set_velocity(int16_t linear, int16_t angular);
void motor_emergency_stop(void);
void motor_enable(bool enable);
void motor_get_current_cmd(int8_t *linear, int8_t *angular);
```

**Differential Drive:**
- `linear`: -100 (rückwärts) bis +100 (vorwärts)
- `angular`: -100 (links drehen) bis +100 (rechts drehen)

### Yaw Rate Controller

PID-basierte Regelung der Drehgeschwindigkeit mit Magnetometer-Feedback.

**Parameter:**

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `YAW_RATE_MAX` | 40°/s | Maximale Drehrate |
| `YAW_KP` | 1.0 | Proportional-Gain |
| `YAW_KI` | 0.3 | Integral-Gain |
| `YAW_KD` | 0.8 | Derivative-Gain |

**Trigger-Mapping:**
- LT (Left Trigger): Drehung nach links (negative Rate)
- RT (Right Trigger): Drehung nach rechts (positive Rate)
- Beide bei 0: Keine Drehung (Feedforward übernimmt)

### Xbox Controller Mapping (Detailliert)

**Button Masks (robot_state.h):**
```c
#define XBOX_BTN_A          0x0001
#define XBOX_BTN_B          0x0002
#define XBOX_BTN_X          0x0004
#define XBOX_BTN_Y          0x0008
#define XBOX_BTN_LB         0x0010
#define XBOX_BTN_RB         0x0020
#define XBOX_BTN_VIEW       0x0040
#define XBOX_BTN_MENU       0x0080
#define XBOX_BTN_LSTICK     0x0100
#define XBOX_BTN_RSTICK     0x0200
#define XBOX_BTN_XBOX       0x0400
```

**D-Pad Werte (Hat Switch):**

| Wert | Richtung |
|------|----------|
| 0 | Neutral |
| 1 | Up |
| 2 | Up-Right |
| 3 | Right |
| 4 | Down-Right |
| 5 | Down |
| 6 | Down-Left |
| 7 | Left |
| 8 | Up-Left |

**Steuerung:**

| Input | Aktion |
|-------|--------|
| Left Stick Y | Vorwärts/Rückwärts |
| Left Stick X | Links/Rechts drehen |
| Right Trigger | Yaw Rate Control (CW) |
| Left Trigger | Yaw Rate Control (CCW) |
| D-Pad UP | Autonomous Mode aktivieren |
| D-Pad DOWN | 180° U-Turn |
| D-Pad LEFT | +90° drehen (CCW) |
| D-Pad RIGHT | -90° drehen (CW) |
| Button A | Sound: Shoot + Emoji |
| Button B | Sound: Machine Gun + Emoji |
| Button X | Emoji: Surprised |
| Button Y | Emoji: Frown |
| **LB + RB** | **Emergency Stop!** |
| **L3 + R3** | **Emergency Stop!** |

**Deadzone:** 10% (~3277 von 32767)

**Manual Override:**
- Stick > 10% ODER Trigger > 10% → Autonomous Mode deaktivieren
- D-Pad-Eingaben deaktivieren Autonomous Mode **nicht**

### micro:bit Button Handling

| Button | Kurzdruck | Langdruck |
|--------|-----------|-----------|
| **Button A** | - | **1s**: BLE Pairing starten/stoppen |
| **Button B** | GPIO-Test weiterschalten | **3s**: Magnetometer-Kalibrierung |

**Display-Animationen:**

| Zustand | Animation |
|---------|-----------|
| Idle | Pulsierendes Herz |
| Pairing/Scanning | Rotierender Punkt |
| Connected | Blinkende Augen |
| Autonomous | Pfeil nach vorne |
| Button A/B/X/Y | Emoji (2s) |
| Kalibrierung | Drehender Punkt |
| Erfolg | Häkchen |
| Fehler | X |

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

### Thread-Architektur

Die Firmware verwendet mehrere Zephyr-Threads für Echtzeit-Verarbeitung:

| Thread | Priorität | Stack | Funktion |
|--------|-----------|-------|----------|
| Main | Default | 3072B | State Machine, Buttons, Display |
| Motor | 2 | 768B | PWM Motor Control (50Hz) |
| BLE Central | 3 | 2048B | Xbox Controller Scan/Connect |
| Sensor | 5 | 1024B | IMU Reading + Orientation (50Hz) |
| Audio | 6 | 512B | Tone Generation |
| IR Sensors | - | - | ADC Obstacle Reading |
| Autonomous Nav | - | - | Heading Control + Avoidance |
| Telemetry | - | - | Binary Packet Streaming (20Hz) |
| Serial Output | - | - | USB Debug Output |

**Message Queues:**

| Queue | Von → Nach | Inhalt |
|-------|------------|--------|
| `motor_cmd_q` | Controller → Motor | linear, angular, emergency_stop |
| `display_state_q` | Any → Display | enum display_state |
| `obstacle_q` | IR Sensors → Nav | left/right detection |
| `sound_q` | Any → Audio | enum sound_event |

---

## Tools & Utilities

### Python Tools

**Installation:**
```bash
pip install pyserial bleak pygame PyOpenGL numpy
```

**1. Telemetry Receiver** (`tools/telemetry_receiver.py`)
```bash
# BLE Telemetrie-Pakete empfangen und parsen
python tools/telemetry_receiver.py
```

**2. Serial Monitor** (`serial_monitor.py`)
```bash
# USB Serial Output anzeigen
python serial_monitor.py
```

**3. IMU Plot** (`imu_orientation/firmware/tools/imu_plot.py`)
```bash
# Echtzeit Roll/Pitch/Heading Plot
python imu_orientation/firmware/tools/imu_plot.py
```

**4. IR Plot** (`imu_orientation/firmware/tools/ir_plot.py`)
```bash
# IR Sensor Werte plotten
python imu_orientation/firmware/tools/ir_plot.py
```

**5. 3D Visualizer** (`imu_orientation/visualizer/`)
```bash
cd imu_orientation/visualizer
pip install -r requirements.txt
python visualizer.py
```

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
