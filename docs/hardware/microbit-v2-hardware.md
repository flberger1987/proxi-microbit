# micro:bit v2 Hardware Dokumentation

## Offizielle Quellen

### Schaltpläne (Schematics)
- **GitHub Repository**: https://github.com/microbit-foundation/microbit-v2-hardware
- **V2.0.0 Schematic PDF**: https://github.com/microbit-foundation/microbit-v2-hardware/blob/main/V2.00/MicroBit_V2.0.0_S_schematic.PDF
- **V2.2.1 Schematic PDF**: https://github.com/microbit-foundation/microbit-v2-hardware/blob/main/V2.21/MicroBit_V2.2.1_nRF52820%20schematic.PDF

### Tech Site
- **Hardware Übersicht**: https://tech.microbit.org/hardware/
- **Schematic Dokumentation**: https://tech.microbit.org/hardware/schematic/

## Hardware Spezifikationen

### Main MCU: Nordic nRF52833
| Parameter | Wert |
|-----------|------|
| Core | ARM Cortex-M4F |
| Frequenz | 64 MHz |
| Flash | 512 KB |
| RAM | 128 KB |
| FPU | Ja (Hardware Floating Point) |
| Bluetooth | BLE 5.0 |

### Interface MCU (DAPLink)
- **V2.00**: Freescale KL27
- **V2.2x**: Nordic nRF52833 oder nRF52820

### Sensoren
- **Accelerometer**: LSM303AGR (3-Achsen)
- **Magnetometer**: LSM303AGR (Kompass)
- **Mikrofon**: MEMS Mikrofon mit LED-Indikator
- **Lautsprecher**: Integriert

### Display
- 5x5 LED Matrix
- Kann auch als Lichtsensor verwendet werden

### Stromversorgung
- USB (5V)
- Batterie (2x AAA über Connector)
- 3V Pad am Edge Connector
- Onboard Regulator: 300mA (190mA für Accessories verfügbar)

## Pinout

### Edge Connector Pins

| Pin | Funktion | GPIO |
|-----|----------|------|
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
| 13 | SCK | P0.17 |
| 14 | MISO | P0.01 |
| 15 | MOSI | P0.13 |
| 16 | GPIO | P1.02 |
| 19 | I2C SCL | P0.26 |
| 20 | I2C SDA | P0.16 |

### Interne Pins
- **UART TX** (zum Interface): P0.06
- **UART RX** (vom Interface): P1.08
- **Speaker**: P0.00
- **Microphone**: P0.05 (Analog)
- **Touch Logo**: P1.04

## Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     micro:bit v2                             │
│                                                              │
│  ┌─────────────┐         ┌─────────────────────────────┐   │
│  │  Interface  │  UART   │      Main MCU               │   │
│  │    MCU      │◄───────►│      nRF52833               │   │
│  │ nRF52833/   │  SWD    │                             │   │
│  │ nRF52820    │◄───────►│  ┌─────┐  ┌────────────┐  │   │
│  │             │         │  │ BLE │  │ Cortex-M4F │  │   │
│  │  DAPLink    │         │  └─────┘  └────────────┘  │   │
│  └──────┬──────┘         │                             │   │
│         │                │  ┌─────────────────────┐    │   │
│         │ USB            │  │ 5x5 LED Matrix      │    │   │
│  ┌──────▼──────┐         │  └─────────────────────┘    │   │
│  │   USB-C     │         │                             │   │
│  │  Connector  │         │  ┌─────┐  ┌─────┐          │   │
│  └─────────────┘         │  │Btn A│  │Btn B│          │   │
│                          │  └─────┘  └─────┘          │   │
│  ┌─────────────┐         │                             │   │
│  │  I2C Bus    │◄───────►│  ┌──────────────────────┐  │   │
│  │ Accelero-   │         │  │ Edge Connector       │  │   │
│  │ meter/Mag   │         │  │ (GPIO, I2C, SPI,     │  │   │
│  └─────────────┘         │  │  Analog, Power)      │  │   │
│                          │  └──────────────────────┘  │   │
│  ┌─────┐ ┌─────┐        │                             │   │
│  │ Mic │ │ Spk │        └─────────────────────────────┘   │
│  └─────┘ └─────┘                                          │
└─────────────────────────────────────────────────────────────┘
```

## Hardware Revisionen

| Version | Interface MCU | Änderungen |
|---------|---------------|------------|
| V2.00 | Freescale KL27 | Erste V2 Version |
| V2.2x | nRF52833 | Selber Chip wie Main MCU |
| V2.21 | nRF52820 | Alternative wegen Chipverfügbarkeit |

Alle V2.2x Versionen verwenden dieselbe DAPLink Firmware.
