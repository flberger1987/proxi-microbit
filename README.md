# ProxiMicro - Kosmos Proxi Hexapod mit micro:bit v2

Zephyr RTOS Firmware für den BBC micro:bit v2 zur Steuerung des Kosmos Proxi Hexapod-Roboters via Xbox/PS5 Wireless Controller.

## Highlights

- **BLE Dual-Role**: Xbox Controller (Central) + Debug/OTA (Peripheral)
- **BLE OTA Updates**: Kabellose Firmware-Updates ohne USB
- **Kalman-Filter**: 2D für Heading+YawRate, 1D für IR-Distanz
- **Autonome Navigation**: Heading-Hold mit IR-Hindernisvermeidung
- **Persistente Einstellungen**: Kalibrierung überlebt Neustart & OTA

## Quick Start

```bash
cd imu_orientation/firmware
./build_flash.sh flash     # Einmalig: Initial-Flash via USB
./build_flash.sh ota       # Updates: Kabellos via BLE
./start_dashboard.sh       # Telemetry Dashboard
```

---

## Systemarchitektur

```
┌────────────────────────────────────────────────────────────────────────────┐
│                              MICRO:BIT V2                                   │
│                            (nRF52833 @ 64MHz)                               │
│                                                                             │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐    │
│  │ BLE Central │   │ BLE Periph. │   │   Sensor    │   │   Motor     │    │
│  │   Thread    │   │   Thread    │   │   Thread    │   │   Thread    │    │
│  │             │   │             │   │             │   │             │    │
│  │ Xbox HID    │   │ NUS Service │   │ IMU 10Hz    │   │ PWM 50Hz    │    │
│  │ Parser      │   │ Telemetry   │   │ IR  20Hz    │   │ H-Bridge    │    │
│  │             │   │ OTA Handler │   │ Kalman      │   │             │    │
│  └──────┬──────┘   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘    │
│         │                 │                 │                 │            │
│         └────────────┬────┴────────┬────────┴────────┬────────┘            │
│                      │             │                 │                      │
│                      ▼             ▼                 ▼                      │
│              ┌─────────────────────────────────────────────┐               │
│              │            Robot State (Shared)             │               │
│              │  - Controller Input  - Orientation          │               │
│              │  - Motor Commands    - IR Distances         │               │
│              │  - Nav State         - Calibration          │               │
│              └─────────────────────────────────────────────┘               │
│                                                                             │
└────────────────────────────────────────────────────────────────────────────┘
        │                    │                              │
        ▼                    ▼                              ▼
┌───────────────┐    ┌───────────────┐              ┌───────────────┐
│ Xbox Wireless │    │   Dashboard   │              │ Kosmos Proxi  │
│  Controller   │    │   (PyQt6)     │              │   Hexapod     │
│               │    │               │              │               │
│  BLE HID      │    │  BLE NUS      │              │  2× Motoren   │
│  16-byte RPT  │    │  Telemetry    │              │  2× IR Sensor │
└───────────────┘    │  OTA Upload   │              └───────────────┘
                     └───────────────┘
```

### Thread-Übersicht

| Thread | Prio | Rate | Aufgabe |
|--------|------|------|---------|
| **BLE Central** | 3 | Event | Xbox Controller Scan, Connect, HID Parsing |
| **Sensor** | 5 | 10Hz | IMU lesen, 2D Kalman (Heading+ω), Mahony AHRS |
| **IR Sensors** | 8 | 20Hz | ADC Differenzmessung, 1D Kalman (Distanz) |
| **Motor** | 2 | 50Hz | PWM Generation, Differential Drive |
| **Telemetry** | 7 | 20Hz | Binary Paket (36 Bytes) via BLE NUS |
| **Autonomous** | 6 | 10Hz | Heading-Hold, Hindernisvermeidung |
| **Main** | - | Event | Display, Buttons, State Machine |

### Signalverarbeitungs-Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SENSOR FUSION                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  LSM303AGR Accelerometer                                                │
│       │                                                                  │
│       ├──► Achsentransformation (Sensor → User Koordinaten)             │
│       │                                                                  │
│       ├──► Roll/Pitch ──► LPF (α=0.15) ──► Telemetry                   │
│       │                                                                  │
│       └──► Reference Gravity ──► EMA (τ≈2s) ──┐                        │
│                                                │                         │
│  LSM303AGR Magnetometer                        │                         │
│       │                                        │                         │
│       ├──► Hard-Iron Kompensation (Offset)    │                         │
│       │                                        │                         │
│       ├──► Soft-Iron Kompensation (Scale)     │                         │
│       │                                        ▼                         │
│       └──► Tilt-kompensiertes Heading ◄───────┘                         │
│                       │                                                  │
│                       ▼                                                  │
│              ┌─────────────────┐                                        │
│              │  2D KALMAN      │◄──── Motor Command (Feedforward)       │
│              │  [θ, ω]         │                                        │
│              │                 │                                        │
│              │  θ' = θ + ω·dt  │                                        │
│              │  ω' = a·ω + b·u │                                        │
│              └────────┬────────┘                                        │
│                       │                                                  │
│                       ▼                                                  │
│              Heading + Yaw Rate ──► Autonomous Nav / Telemetry          │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  IR Sensoren (ADC)                                                      │
│       │                                                                  │
│       ├──► Differenzmessung (LED on - LED off)                          │
│       │                                                                  │
│       ├──► IIR Low-Pass (fc=10Hz)                                       │
│       │                                                                  │
│       ├──► Bias-Kompensation (Startup-Kalibrierung)                     │
│       │                                                                  │
│       ├──► ADC → mm (Lookup-Tabelle + Interpolation)                    │
│       │                                                                  │
│       └──► 1D Kalman ──► Distanz (mm) ──► Autonomous Nav                │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## OTA Update System

**Boot-Pointer Dual-Slot System** (kein MCUboot):

```
Flash Layout (512KB):
┌──────────────────┬──────────┐
│ Mini-Bootloader  │   8 KB   │  Boot-Entscheidung, CRC-Check
├──────────────────┼──────────┤
│ Slot A (Primary) │  244 KB  │  Aktive Firmware
├──────────────────┼──────────┤
│ Slot B (Backup)  │  244 KB  │  OTA-Ziel / Fallback
├──────────────────┼──────────┤
│ Boot Control     │   4 KB   │  Aktiver Slot, CRCs, Flags
├──────────────────┼──────────┤
│ Settings (NVS)   │   8 KB   │  Kalibrierung, PID, BLE Bonds
└──────────────────┴──────────┘
```

- **Pointer-Switch** statt Firmware-Kopieren
- **Auto-Fallback** nach 3 Boot-Fehlern
- **Resume** nach BLE-Disconnect

---

## Signalverarbeitung

### Problem: Hexapod-Gang verursacht IMU-Oszillationen

**FFT-Analyse** identifizierte Gait-Frequenz bei **~1.27 Hz** mit ±7° Heading-Drift.

### Lösung: Mehrstufige Filterung

| Sensor | Filter | Parameter | Zweck |
|--------|--------|-----------|-------|
| Accelerometer | Reference Gravity EMA | τ≈2s (α=0.05) | Filtert Gait-Oszillationen |
| Magnetometer | 2D Kalman [θ, ω] | Q=0.5/10, R=3 | Heading + Yaw Rate |
| IR Sensoren | IIR + 1D Kalman | fc=10Hz, Q=100, R=400 | Distanz-Glättung |

**2D Kalman** schätzt Yaw Rate ohne Gyroskop durch Motor-Feedforward:
```
θ' = θ + ω×dt
ω' = 0.67×ω + 0.13×motor_cmd
```

---

## Hardware

| Komponente | Details |
|------------|---------|
| MCU | nRF52833 (Cortex-M4F, 64MHz, 512KB Flash, 128KB RAM) |
| IMU | LSM303AGR (Accel + Mag, I2C) |
| IR | 2× Reflexionssensoren (P0, P1), LED Enable (P12) |
| Motoren | H-Brücke PWM (P13-P16) |
| Display | 5×5 LED Matrix |
| Audio | PWM Speaker (P0.00) |

---

## Steuerung

| Input | Aktion |
|-------|--------|
| Left Stick | Fahren + Drehen |
| Trigger L/R | Yaw Rate Control |
| D-Pad UP | Autonomous Mode |
| D-Pad DOWN/L/R | 180°/±90° Turn |
| LB+RB | Emergency Stop |
| Button B (3s) | Magnetometer-Kalibrierung |

---

## Tools

| Tool | Funktion |
|------|----------|
| `./build_flash.sh ota` | BLE Firmware Update |
| `./start_dashboard.sh` | PyQt6 Telemetry GUI |
| `tools/analyze_telemetry.py` | FFT-Analyse für Gait-Frequenz |
| `tools/ota_upload.py` | Standalone OTA Upload |

---

## Dokumentation

Ausführliche technische Details: **[CLAUDE.md](CLAUDE.md)**

## Lizenz

Apache-2.0

---
*Stand: 2026-01-30*
