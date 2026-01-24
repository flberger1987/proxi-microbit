# CODAL SDK für micro:bit v2

## Übersicht

CODAL (Component Oriented Device Abstraction Layer) ist das offizielle Runtime von Lancaster University für den micro:bit. Es bietet eine C++ API mit Fiber-basiertem kooperativem Multitasking.

## Offizielle Quellen

- **Tech Site**: https://tech.microbit.org/software/runtime/
- **CODAL micro:bit v2**: https://github.com/lancaster-university/codal-microbit-v2
- **micro:bit v2 Samples**: https://github.com/lancaster-university/microbit-v2-samples
- **API Dokumentation**: https://lancaster-university.github.io/microbit-docs/

## Installation

### Voraussetzungen

```bash
# macOS
brew install cmake ninja arm-none-eabi-gcc python3

# Linux (Ubuntu/Debian)
sudo apt install cmake ninja-build gcc-arm-none-eabi python3

# Windows: ARM GCC von developer.arm.com installieren
```

### Repository klonen

```bash
git clone https://github.com/lancaster-university/microbit-v2-samples
cd microbit-v2-samples

# Submodule initialisieren
git submodule update --init
```

## Build-System

### Projekt bauen

```bash
# Erstes Build (holt Dependencies automatisch)
python build.py

# Output: MICROBIT.hex in build/
```

### Clean Build

```bash
rm -rf build/
python build.py
```

## Code-Struktur

### main.cpp Beispiel

```cpp
#include "MicroBit.h"

MicroBit uBit;

int main()
{
    uBit.init();

    while(true)
    {
        uBit.display.scroll("HELLO!");
        uBit.sleep(1000);
    }
}
```

### Mit Buttons

```cpp
#include "MicroBit.h"

MicroBit uBit;

void onButtonA(MicroBitEvent e)
{
    uBit.display.print("A");
}

void onButtonB(MicroBitEvent e)
{
    uBit.display.print("B");
}

int main()
{
    uBit.init();

    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, onButtonB);

    while(true)
    {
        uBit.sleep(1000);
    }
}
```

## Fiber-basiertes Multitasking

CODAL verwendet kooperative Fibers statt preemptiver Threads. Fibers sind leichtgewichtig und teilen sich einen Stack.

### Fiber erstellen

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
        uBit.sleep(500);   // Gibt Control ab
    }
}

int main()
{
    uBit.init();

    // Fibers starten
    create_fiber(task1);
    create_fiber(task2);

    // Main Fiber muss auch etwas tun oder schlafen
    while(true)
    {
        uBit.sleep(10000);
    }
}
```

### Fiber-Kommunikation mit Events

```cpp
#include "MicroBit.h"

MicroBit uBit;

#define MY_EVENT_ID 1234
#define MY_EVENT_VALUE 1

void producer()
{
    while(true)
    {
        uBit.sleep(2000);
        MicroBitEvent evt(MY_EVENT_ID, MY_EVENT_VALUE);
    }
}

void consumer(MicroBitEvent e)
{
    uBit.display.print("!");
}

int main()
{
    uBit.init();

    uBit.messageBus.listen(MY_EVENT_ID, MY_EVENT_VALUE, consumer);
    create_fiber(producer);

    while(true)
    {
        uBit.sleep(10000);
    }
}
```

## Hardware-Zugriff

### LED Display

```cpp
uBit.display.print("X");           // Einzelnes Zeichen
uBit.display.scroll("Hello");      // Scrollender Text
uBit.display.image.setPixelValue(2, 2, 255);  // Pixel setzen
```

### Accelerometer

```cpp
int x = uBit.accelerometer.getX();
int y = uBit.accelerometer.getY();
int z = uBit.accelerometer.getZ();

// Gesten
if (uBit.accelerometer.getGesture() == MICROBIT_ACCELEROMETER_EVT_SHAKE)
{
    uBit.display.print("!");
}
```

### GPIO Pins

```cpp
// Digital Output
uBit.io.P0.setDigitalValue(1);

// Digital Input
int val = uBit.io.P1.getDigitalValue();

// Analog Input (0-1023)
int analog = uBit.io.P2.getAnalogValue();

// PWM Output
uBit.io.P0.setAnalogValue(512);  // 50% Duty Cycle
```

### I2C

```cpp
char cmd[2] = {0x00, 0xFF};
uBit.i2c.write(0x1D, cmd, 2);

char data[6];
uBit.i2c.read(0x1D, data, 6);
```

### Speaker & Microphone (V2)

```cpp
// Sound abspielen
uBit.audio.soundExpressions.play(ManagedString("giggle"));

// Mikrofon Level
int level = uBit.audio.levelSPL->getValue();
```

## Unterschied zu Zephyr

| Feature | CODAL | Zephyr |
|---------|-------|--------|
| Scheduling | Kooperative Fibers | Preemptive Threads |
| Komplexität | Einfacher | Mehr Features |
| micro:bit API | Nativ | Generic |
| Bluetooth | Integriert | Konfigurierbar |
| Footprint | Kleiner | Größer |
| RTOS Features | Basis | Vollständig |

## Wann CODAL verwenden?

- Einfache bis mittlere Projekte
- micro:bit-spezifische Features wichtig
- Schneller Einstieg gewünscht
- Kompatibilität mit MakeCode wichtig

## Wann Zephyr bevorzugen?

- Echte preemptive Threads benötigt
- Komplexe Synchronisation
- Andere Zephyr-unterstützte Hardware
- Industriestandard-RTOS gewünscht
