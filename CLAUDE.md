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
CONFIG_MAIN_STACK_SIZE=4096

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

# Persistent Storage (Bonding, Calibration, PID)
CONFIG_SETTINGS=y
CONFIG_NVS=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_FLASH_PAGE_LAYOUT=y

# OTA Dual-Slot System
CONFIG_USE_DT_CODE_PARTITION=y
CONFIG_CRC=y
CONFIG_REBOOT=y

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

# Size optimization
CONFIG_SIZE_OPTIMIZATIONS=y
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

## BLE OTA Dual-Slot Bootloader System

Das Projekt verwendet ein eigenes **Boot-Pointer OTA System** für kabellose Firmware-Updates über Bluetooth. Kein MCUboot erforderlich.

### Flash-Layout (nRF52833 512KB)

```
┌─────────────────────┬────────────┬─────────┐
│ Mini-Bootloader     │ 0x00000    │ 8 KB    │
│ (Boot-Pointer)      │ - 0x02000  │         │
├─────────────────────┼────────────┼─────────┤
│ Slot A (Primary)    │ 0x02000    │ 244 KB  │
│                     │ - 0x3F000  │         │
├─────────────────────┼────────────┼─────────┤
│ Slot B (Secondary)  │ 0x3F000    │ 244 KB  │
│                     │ - 0x7C000  │         │
├─────────────────────┼────────────┼─────────┤
│ Boot Control Block  │ 0x7C000    │ 4 KB    │
│ (Pointer + Flags)   │ - 0x7D000  │         │
├─────────────────────┼────────────┼─────────┤
│ OTA State           │ 0x7D000    │ 4 KB    │
│ (Transfer Progress) │ - 0x7E000  │         │
├─────────────────────┼────────────┼─────────┤
│ Settings (NVS)      │ 0x7E000    │ 8 KB    │
│ (Calibration, PID,  │ - 0x80000  │         │
│  BLE Bonds)         │            │         │
└─────────────────────┴────────────┴─────────┘
```

### Boot-Pointer Konzept

Anstatt Firmware zu kopieren, wird nur ein **Pointer** auf den aktiven Slot umgeschaltet:

1. **OTA-Update** schreibt neue Firmware in den **inaktiven** Slot
2. Nach erfolgreicher **CRC32-Validierung** wird der Boot-Pointer umgeschaltet
3. **Automatischer Fallback**: Nach 3 fehlgeschlagenen Boot-Versuchen → Rollback zum anderen Slot
4. **Settings bleiben erhalten**: Kalibrierung, PID-Parameter, BLE-Bonds in separater Partition

### Bootloader Boot-Entscheidung

Der Mini-Bootloader (8KB) trifft beim Start folgende Entscheidungen:

```
Boot-Entscheidung:
  1. Boot Control Block lesen und validieren
  2. CRC32 beider Slots prüfen
  3. Slot wählen:
     ├─ Beide gültig → Höhere Build-Nummer gewinnt
     ├─ Nur einer gültig → Diesen verwenden
     └─ Keiner gültig → Fallback zu Slot A
  4. boot_count prüfen:
     └─ >=3 → Auf anderen Slot wechseln (wenn gültig)
  5. boot_count inkrementieren
  6. Vector Table validieren (SP in RAM, Reset in Slot)
  7. Zu Firmware springen
```

**Erfolgreicher Boot:**
- Firmware ruft `boot_control_confirm_boot()` auf
- Setzt `boot_count` auf 0 zurück

**Fallback-Szenarien:**
- Firmware crasht vor `confirm_boot()` → boot_count steigt bei jedem Reset
- Nach 3 Versuchen → Automatischer Wechsel zum anderen Slot
- Falls anderer Slot auch ungültig → Trotzdem Slot A versuchen

### Boot Control Block Struktur

```c
struct boot_control {
    uint32_t magic;           /* 0xB007F1E0 ("BOOT FILE" in leetspeak) */
    uint8_t  version;         /* Struktur-Version (aktuell: 2) */
    uint8_t  active_slot;     /* 0 = Slot A, 1 = Slot B */
    uint8_t  slot_a_valid;    /* 1 = gültige Firmware in Slot A */
    uint8_t  slot_b_valid;    /* 1 = gültige Firmware in Slot B */
    uint8_t  boot_count;      /* Fallback-Zähler (>=3 → Rollback) */
    uint8_t  fallback_reason; /* BOOT_FALLBACK_* Code */
    uint8_t  last_boot_slot;  /* Diagnostik: tatsächlich gebooteter Slot */
    uint8_t  reserved;        /* Alignment padding */
    uint32_t slot_a_crc;      /* CRC32 der Firmware in Slot A */
    uint32_t slot_b_crc;      /* CRC32 der Firmware in Slot B */
    uint32_t slot_a_size;     /* Firmware-Größe in Slot A (Bytes) */
    uint32_t slot_b_size;     /* Firmware-Größe in Slot B (Bytes) */
    uint32_t slot_a_version;  /* Build-Nummer in Slot A */
    uint32_t slot_b_version;  /* Build-Nummer in Slot B */
    uint32_t checksum;        /* CRC32 dieser Struktur */
};
```

**Fallback Reason Codes:**
- `0` NONE - Normaler Boot
- `1` CRC_FAIL - Primärer Slot CRC-Fehler
- `2` VERSION - Niedrigere Version (anderer Slot ungültig)
- `3` BOOT_COUNT - Zu viele Boot-Versuche (>=3)

### OTA State Machine

```
          ┌─────────────────────────────────────────┐
          │                                         │
          ▼                                         │
    ┌──────────┐    INIT      ┌─────────┐    ┌─────┴─────┐
    │   IDLE   │─────────────►│  ERASE  │───►│ TRANSFER  │
    └──────────┘              └─────────┘    └───────────┘
          ▲                                       │ │
          │    ┌──────────┐                       │ │ disconnect
          │◄───│  ABORT   │◄──────────────────────┘ │
          │    └──────────┘                         ▼
          │                                  ┌───────────┐
          │    ┌──────────┐    QUERY         │ SUSPENDED │
          │◄───│ VALIDATE │◄─────────────────┴───────────┘
          │    └────┬─────┘         reconnect
          │         │ CRC OK
          │         ▼
          │    ┌──────────┐
          └────│  COMMIT  │───► REBOOT
               └──────────┘
```

**Zustände (enum ota_state):**

| Zustand | Beschreibung |
|---------|--------------|
| `IDLE` | Bereit für neues OTA |
| `BACKUP` | Sendet aktuelle Firmware an Host |
| `ERASE` | Löscht Ziel-Slot (Page für Page) |
| `TRANSFER` | Empfängt Datenblöcke |
| `SUSPENDED` | Verbindung verloren, wartet auf Resume |
| `VALIDATE` | Prüft CRC32 der empfangenen Firmware |
| `COMMIT` | Boot-Pointer umgeschaltet, Reboot |
| `ABORT` | Abgebrochen, zurück zu IDLE |

### OTA-Protokoll

**BLE-basiert über Nordic UART Service (NUS).**

Alle OTA-Pakete beginnen mit `0xF0` (Magic Byte) gefolgt vom Command-Byte.

**Commands (Host → Device):**

| Command | Code | Payload | Beschreibung |
|---------|------|---------|--------------|
| `OTA_INIT` | 0x01 | size(4B), crc32(4B), version(16B) | Start OTA, Ziel-Slot löschen |
| `OTA_DATA` | 0x02 | block_num(2B), crc16(2B), data(224B) | Firmware-Block |
| `OTA_QUERY` | 0x03 | - | Status abfragen (für Resume) |
| `OTA_ABORT` | 0x04 | - | OTA abbrechen |
| `OTA_VALIDATE` | 0x05 | - | Finale CRC32-Validierung starten |
| `OTA_BACKUP_REQ` | 0x06 | - | Firmware-Backup anfordern |
| `OTA_BACKUP_ACK` | 0x07 | - | Nächsten Backup-Block anfordern |

**Responses (Device → Host):**

| Response | Code | Payload | Beschreibung |
|----------|------|---------|--------------|
| `RESP_OK` | 0x00 | - | Generischer Erfolg |
| `RESP_READY` | 0x01 | status struct | Bereit für Transfer |
| `RESP_ACK` | 0x02 | block_num(2B) | Block erfolgreich empfangen |
| `RESP_NAK` | 0x03 | block_num(2B), error(1B) | Block-Fehler, bitte wiederholen |
| `RESP_STATUS` | 0x04 | status struct | Aktueller Status (für Resume) |
| `RESP_VALID_OK` | 0x05 | - | Validierung OK → Reboot |
| `RESP_VALID_FAIL` | 0x06 | expected(4B), actual(4B) | CRC32 stimmt nicht |
| `RESP_BACKUP_DATA` | 0x07 | block_num(2B), crc16(2B), data(224B) | Backup-Block |
| `RESP_BACKUP_DONE` | 0x08 | size(4B), crc32(4B) | Backup vollständig |
| `RESP_ERROR` | 0x10 | error_code(1B) | Fehler aufgetreten |

**Error Codes:**
- `0x01` CRC - CRC-Prüfsumme falsch
- `0x02` SEQUENCE - Falsche Block-Nummer
- `0x03` SIZE - Firmware zu groß
- `0x04` FLASH - Flash-Schreibfehler
- `0x05` STATE - Ungültiger Zustand für Operation
- `0x06` TIMEOUT - Zeitüberschreitung
- `0x07` INVALID - Ungültiges Paket

### OTA Transfer-Details

**Block-Struktur:**
- Block-Größe: 224 Bytes (OTA_BLOCK_SIZE)
- Paket-Format: `[0xF0][cmd][block_num:2LE][crc16:2LE][data:224]`
- Letzter Block: Mit 0xFF aufgefüllt (Flash-Löschwert)
- CRC16: CCITT (Polynom 0x1021, Initial 0xFFFF)
- Block-Nummerierung: 1-basiert (1, 2, 3, ..., total_blocks)

**Flash-Timing (BLE-Aware):**
- Schreibt in 256-Byte Chunks (~2.6ms pro Chunk)
- 5ms Pause zwischen Chunks (BLE Events verarbeiten)
- 30ms Pause nach voller Page (4KB)
- 50ms Pause zwischen Page-Erases

**Resume nach Disconnect:**
1. Transfer unterbrochen → SUSPENDED
2. Host reconnectet → sendet QUERY
3. Device antwortet mit last_block
4. Host sendet ab last_block+1 weiter

**Thread-Management während OTA:**
- Suspendiert: sensors, motor, ir_sensors, autonav, telemetry
- Aktiv bleiben: main (Display), BLE Stack

### Build & Flash Befehle

```bash
cd imu_orientation/firmware

# Nur bauen
./build_flash.sh

# Initial-Flash (LÖSCHT Settings!)
./build_flash.sh flash

# OTA Update via BLE (empfohlen für Updates)
./build_flash.sh ota

# OTA mit Backup vorher
./build_flash.sh ota --backup

# USB-Flash ohne Settings zu löschen
./build_flash.sh flash-app

# Backup der aktuellen Firmware
./build_flash.sh backup

# OTA-Status abfragen
./build_flash.sh status

# Bootloader separat bauen
./build_flash.sh bootloader

# Build-Verzeichnis löschen
./build_flash.sh clean
```

### Datensicherheit bei Updates

| Update-Methode | Kalibrierung | PID-Params | BLE-Bonds |
|----------------|--------------|------------|-----------|
| `./build_flash.sh ota` | ✅ Erhalten | ✅ Erhalten | ✅ Erhalten |
| `./build_flash.sh flash-app` | ✅ Erhalten | ✅ Erhalten | ✅ Erhalten |
| `./build_flash.sh flash` | ❌ GELÖSCHT | ❌ GELÖSCHT | ❌ GELÖSCHT |

**Empfohlener Workflow:**
1. **Erstmaliges Setup:** `./build_flash.sh flash` (nur einmal)
2. **Kalibrierung:** Button B 3s halten
3. **Alle weiteren Updates:** `./build_flash.sh ota`

### Python OTA-Tool

```bash
# Installation
pip install bleak tqdm intelhex

# OTA Update (mit Backup)
python tools/ota_upload.py build/app/zephyr.bin

# OTA Update ohne Backup (schneller)
python tools/ota_upload.py build/app/zephyr.bin --no-backup

# Nur Backup der aktuellen Firmware
python tools/ota_upload.py --backup

# Status abfragen
python tools/ota_upload.py --status

# Resume nach Abbruch (ab Block 50 weitermachen)
python tools/ota_upload.py build/app/zephyr.bin --resume 50
```

**Features:**
- Automatisches Backup vor Update (deaktivierbar mit `--no-backup`)
- Resume nach Verbindungsabbruch (mit `--resume <block>`)
- Unterstützt `.hex` (Intel HEX) und `.bin` (raw binary) Formate
- Exponentielles Backoff bei Block-Fehlern (3 Retries)
- Progress-Bar mit Geschwindigkeitsanzeige

### Standard: DAPLink

Der micro:bit verwendet DAPLink als Interface-Firmware:
- **Drag & Drop**: `.hex` Datei auf `MICROBIT` Laufwerk kopieren
- **CMSIS-DAP**: Debugging über USB
- **CDC Serial**: Serielle Konsole (115200 Baud)

**Maintenance Mode** (bei Bootloader-Problemen):
1. RESET-Taste gedrückt halten
2. USB-Kabel anschließen
3. Laufwerk erscheint als `MAINTENANCE`

---

## Projekt-Struktur

```
ProxiMicro/
├── CLAUDE.md                           # Diese Datei (Developer Reference)
├── README.md                           # Projekt-Übersicht & Quick Start
├── start_dashboard.sh                  # Telemetry Dashboard Startskript
│
├── tools/
│   ├── ota_upload.py                   # BLE OTA Upload Tool
│   ├── telemetry_receiver.py           # BLE Telemetry Empfänger (bleak)
│   ├── analyze_telemetry.py            # CSV Telemetrie-Analyse
│   ├── heading_simulation.py           # PID Tuning Simulation
│   │
│   └── telemetry_gui/                  # PyQt6 Telemetry Dashboard
│       ├── dashboard.py                # Haupt-Dashboard-Fenster
│       ├── ble_bridge.py               # BLE-Qt Async Bridge
│       └── widgets/
│           ├── compass.py              # Kompass-Anzeige
│           ├── heading_graph.py        # Heading-Verlauf Graph
│           ├── ir_sensors.py           # IR Distanz-Anzeige
│           ├── motor_output.py         # Motor-Balken
│           ├── yaw_rate.py             # Yaw Rate Gauge
│           ├── orientation.py          # Roll/Pitch Anzeige
│           ├── status_panel.py         # Status-Panel
│           ├── thread_stats.py         # Thread Statistiken
│           ├── pid_tuning.py           # PID Parameter Tuning
│           ├── variance_display.py     # Sensor-Varianz Anzeige
│           └── log_analysis.py         # Log-Analyse Widget
│
├── imu_orientation/
│   ├── firmware/
│   │   ├── src/                        # Quellcode (20 .c + 19 .h)
│   │   │   ├── main.c                  # Hauptprogramm, State Machine
│   │   │   ├── robot_state.c/h         # Globaler Roboter-Zustand
│   │   │   ├── sensors.c/h             # LSM303AGR + 2D Kalman
│   │   │   ├── orientation.c/h         # Roll/Pitch/Heading
│   │   │   ├── mahony_filter.c/h       # Mahony AHRS (Quaternion)
│   │   │   ├── ble_central.c/h         # BLE Central (Xbox)
│   │   │   ├── ble_output.c/h          # Nordic UART Service
│   │   │   ├── smp_bt.c/h              # BLE Stack Init
│   │   │   ├── hid_parser.c/h          # Xbox/PS5 HID Parser
│   │   │   ├── motor_driver.c/h        # PWM H-Brücke
│   │   │   ├── autonomous_nav.c/h      # Autonome Navigation
│   │   │   ├── yaw_controller.c/h      # PID Yaw Rate
│   │   │   ├── ir_sensors.c/h          # IR + 1D Kalman
│   │   │   ├── audio.c/h               # PWM Tongenerierung
│   │   │   ├── telemetry.c/h           # Binary Telemetrie
│   │   │   ├── serial_output.c/h       # USB Debug
│   │   │   ├── boot_control.c/h        # Boot-Pointer Verwaltung
│   │   │   ├── ota_update.c/h          # OTA State Machine
│   │   │   ├── motor_test.c/h          # GPIO Test
│   │   │   └── sysid.c/h               # System Identification
│   │   │
│   │   ├── bootloader/                 # Mini-Bootloader (8KB)
│   │   │   ├── src/main.c              # Boot-Pointer Logik
│   │   │   ├── boards/
│   │   │   │   └── bbc_microbit_v2.overlay
│   │   │   ├── prj.conf                # Minimale Zephyr-Config
│   │   │   └── CMakeLists.txt
│   │   │
│   │   ├── boards/
│   │   │   └── bbc_microbit_v2.overlay # Flash-Layout + PWM
│   │   │
│   │   ├── build/                      # Build-Outputs (gitignored)
│   │   │   ├── app/zephyr.bin          # Firmware Binary
│   │   │   └── bootloader/zephyr.hex   # Bootloader HEX
│   │   │
│   │   ├── prj.conf                    # Zephyr Haupt-Konfiguration
│   │   ├── CMakeLists.txt              # Build-Konfiguration
│   │   └── build_flash.sh              # Build & Flash Script
│   │
│   └── visualizer/                     # 3D Orientierungs-Visualisierer
│       ├── visualizer.py
│       └── cube_renderer.py
│
└── docs/
    └── hardware/
        └── xbox-controller-ble-hid.md  # Xbox BLE HID Protokoll
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
| `AUTONAV_HEADING_HOLD` | 1 | Fährt vorwärts mit kontinuierlicher Heading-Korrektur |
| `AUTONAV_TURNING` | 2 | Dreht auf neues Ziel-Heading (in-place Rotation) |
| `AUTONAV_SCANNING` | 3 | Hindernis erkannt: scannt links/rechts für besten Pfad |

**Hindernisvermeidung (Kalman-gefiltert):**
- **> 400mm**: Volle Geschwindigkeit (100%), nur Heading-Korrektur
- **250-400mm**: Langsamer fahren, Hindernis nahe
- **< 250mm**: Stoppen und scannen (AUTONAV_SCANNING)

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
    uint8_t  nav_state;          /* 0=DISABLED, 1=HEADING_HOLD, 2=TURNING, 3=SCANNING */
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
| 20 | B | nav_state | enum autonav_state (0-3) |
| 21 | B | flags | Bit 0: autonav, Bit 1: motors |
| 22 | H | target_heading_x10 | Autonomous Nav Target |
| 24 | hhh | raw_ax/ay/az | **Roh** Accel (milli-g, User coords) |
| 30 | hhh | raw_mx/my/mz | **Roh** Mag (milli-Gauss, User coords) |

**Kalman-Filter Details:**
- **Heading + Yaw Rate**: 2D Kalman mit Prozessmodell `θ' = θ + ω·dt`, `ω' = a·ω + b·motor_cmd`
- **IR Distanz**: 1D Kalman für Glättung der ADC→mm Konvertierung

**Test-Script:** `tools/telemetry_receiver.py` (benötigt `pip install bleak`)

### Sensorfusion & Kalman-Filter

**Heading-Drehrichtung (Verifiziert 2026-01-26):**

| Drehung | Heading | Motor | Kalman ω |
|---------|---------|-------|----------|
| **CW** (rechts, im Uhrzeigersinn) | **sinkt** | angular > 0 | negativ |
| **CCW** (links, gegen Uhrzeiger) | **steigt** | angular < 0 | positiv |

**Implementierung:**
- `MOTOR_K = -0.4` in sensors.c (negativ weil CW → heading sinkt)
- `sensors_get_yaw_rate()` gibt Motor-Konvention zurück (positiv = CW)
- Yaw Controller: RT = positiv = "drehe rechts" (CW)

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

Die Magnetometer-Kalibrierung kompensiert zwei Arten von Störungen:

**1. Hard-Iron (Offset):**
- Verursacht durch permanente Magnete in der Nähe
- Verschiebt den Mittelpunkt der Messungen
- Kompensation: `m_centered = m_raw - offset`

**2. Soft-Iron (Skalierung):**
- Verursacht durch ferromagnetische Materialien (Eisen, Nickel)
- Verzerrt die Kugel zu einem Ellipsoid
- Kompensation: `m_calibrated = scale * m_centered`

**Kalibrierungs-Algorithmus:**
```
1. Sammle 300 Samples (30s bei 10Hz)
2. Berechne Min/Max für jede Achse

Hard-Iron:
   offset = (max + min) / 2

Soft-Iron (Ellipsoid → Sphäre):
   range_x = max_x - min_x
   range_y = max_y - min_y
   range_z = max_z - min_z
   avg_range = (range_x + range_y + range_z) / 3
   scale_x = avg_range / range_x
   scale_y = avg_range / range_y
   scale_z = avg_range / range_z
```

**Kalibrierungs-Struktur (orientation.h):**
```c
struct mag_calibration {
    float offset_x, offset_y, offset_z;  /* Hard-Iron */
    float scale_x, scale_y, scale_z;     /* Soft-Iron */
    uint8_t version;                      /* Flash-Kompatibilität */
    bool valid;
};
```

**Kalibrierung starten:**
1. **Long-press Button B** (3 Sekunden gedrückt halten)
2. **30 Sekunden** lang den micro:bit durch **alle Orientierungen** drehen
3. Dabei alle Achsen abdecken (wie eine Kugel abrollen - wichtig für Soft-Iron!)
4. Kalibrierung wird automatisch im Flash gespeichert

**Serielle Ausgabe nach Kalibrierung:**
```
CAL,DONE
  Hard-Iron (offset): 123.4, -56.7, 89.0
  Soft-Iron (scale):  1.05, 0.98, 0.97
  Ranges: X=456.0, Y=478.0, Z=489.0 (avg=474.3)
CAL: Saved to flash
```

**Wichtig:**
- Kalibrierung wird in NVS-Partition @ 0x7E000 gespeichert (überlebt Neustart)
- **`./build_flash.sh flash` löscht die Kalibrierung!** (chip erase)
- **`./build_flash.sh ota` und `flash-app` behalten die Kalibrierung!**
- Nach Verlust: Neu kalibrieren mit Long-press Button B (3s)

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

**Proximity-Warnung (Display-Blink):**
- Ersetzt akustische Näherungs-Warnung durch visuelles Blinken
- ≥450mm: konstant (kein Blinken)
- 150mm: 20Hz Blinken (schnellste Rate)
- Linearer Übergang zwischen 450mm und 150mm

### Build & Flash

```bash
cd imu_orientation/firmware

# Nur bauen
./build_flash.sh

# Initial-Flash (Bootloader + App) - LÖSCHT Settings!
./build_flash.sh flash

# OTA Update via BLE (empfohlen für alle weiteren Updates)
./build_flash.sh ota

# USB-Flash ohne Settings zu löschen
./build_flash.sh flash-app
```

### Aktuelle Konfiguration

- **Boot-System**: Custom Mini-Bootloader mit Boot-Pointer (8KB)
- **OTA**: BLE-basiert über Nordic UART Service
- **Firmware-Größe**: ~244KB (99.98% von 244KB Slot)
- **Settings**: 8KB NVS (Kalibrierung, PID, BLE-Bonds)

**Wichtig:** `./build_flash.sh flash` löscht alle Settings! Für Updates immer `ota` oder `flash-app` verwenden.

### Thread-Architektur

Die Firmware verwendet mehrere Zephyr-Threads für Echtzeit-Verarbeitung:

| Thread | Priorität | Stack | Funktion |
|--------|-----------|-------|----------|
| Main | Default | 4096B | State Machine, Buttons, Display |
| Motor | 2 | 768B | PWM Motor Control (50Hz) |
| BLE Central | 3 | 2048B | Xbox Controller Scan/Connect |
| Sensor | 5 | 1024B | IMU Reading + 2D Kalman (50Hz) |
| Audio | 6 | 512B | Tone Generation |
| IR Sensors | - | - | ADC + 1D Kalman |
| Autonomous Nav | - | - | Heading Control + Avoidance |
| Telemetry | - | - | Binary Packet Streaming (20Hz) |
| OTA Update | - | - | Flash Write + Validation |

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
pip install pyserial bleak PyQt6 tqdm intelhex numpy
```

**1. OTA Upload Tool** (`tools/ota_upload.py`)
```bash
# Firmware via BLE updaten
python tools/ota_upload.py build/app/zephyr.bin

# Mit Backup vorher
python tools/ota_upload.py build/app/zephyr.bin --backup

# Nur Backup der aktuellen Firmware
python tools/ota_upload.py --backup

# Status abfragen
python tools/ota_upload.py --status
```

**2. Telemetry Dashboard (GUI)** (`start_dashboard.sh`)
```bash
# PyQt6 Dashboard starten
./start_dashboard.sh

# Oder direkt:
python -m tools.telemetry_gui.dashboard
```

**3. Telemetry Receiver (Console)** (`tools/telemetry_receiver.py`)
```bash
# BLE Telemetrie-Pakete empfangen
python tools/telemetry_receiver.py

# Mit CSV-Logging:
python tools/telemetry_receiver.py --log telemetry.csv
```

**4. Telemetry Analyse** (`tools/analyze_telemetry.py`)
```bash
# CSV-Datei analysieren und plotten
python tools/analyze_telemetry.py telemetry.csv
```

**5. 3D Visualizer** (`imu_orientation/visualizer/`)
```bash
cd imu_orientation/visualizer
python visualizer.py
```

---

## Quick Start

### 1. Zephyr installieren
```bash
pip install west
west init ~/zephyrproject && cd ~/zephyrproject && west update
source ~/zephyrproject/zephyr/zephyr-env.sh
```

### 2. Firmware bauen
```bash
cd /path/to/ProxiMicro/imu_orientation/firmware
./build_flash.sh
```

### 3. Initial-Flash (einmalig)
```bash
./build_flash.sh flash
# Danach: Magnetometer kalibrieren (Button B 3s halten)
```

### 4. Updates via BLE OTA
```bash
./build_flash.sh ota
```

### 5. Telemetry Dashboard
```bash
cd /path/to/ProxiMicro
./start_dashboard.sh
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
