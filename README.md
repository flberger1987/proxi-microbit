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

## Architektur

```
Xbox Controller ──BLE HID──► micro:bit v2 ──PWM──► Hexapod Motoren
                                 │
Dashboard/OTA ───BLE NUS────────┘
```

## OTA Update System

**Boot-Pointer Dual-Slot System** (kein MCUboot):
- Slot A/B (je 244KB) für Firmware
- Pointer-Switch statt Kopieren
- Auto-Fallback nach 3 Boot-Fehlern
- Settings (Kalibrierung, PID) bleiben bei OTA erhalten

## Signalverarbeitung

### Problem: Hexapod-Gang verursacht IMU-Oszillationen

**FFT-Analyse** identifizierte Gait-Frequenz bei **~1.27 Hz** mit ±7° Heading-Drift.

### Lösung: Mehrstufige Filterung

| Sensor | Filter | Zweck |
|--------|--------|-------|
| Accelerometer | Reference Gravity EMA (τ≈2s) | Stabile Laufebene, filtert Gait |
| Magnetometer | 2D Kalman [θ, ω] | Heading + Yaw Rate aus Mag + Motor-Modell |
| IR Sensoren | LPF + 1D Kalman | Rauschunterdrückung + Glättung |

**2D Kalman** schätzt Yaw Rate ohne Gyroskop durch Motor-Feedforward:
```
θ' = θ + ω×dt
ω' = a×ω + b×motor_cmd
```

## Hardware

| Komponente | Details |
|------------|---------|
| MCU | nRF52833 (Cortex-M4F, 64MHz, 512KB Flash) |
| IMU | LSM303AGR (Accel + Mag) |
| IR | 2× Reflexionssensoren (P0, P1, P12=Enable) |
| Motoren | H-Brücke (P13-P16) |

## Steuerung

| Input | Aktion |
|-------|--------|
| Left Stick | Fahren + Drehen |
| Trigger L/R | Yaw Rate Control |
| D-Pad UP | Autonomous Mode |
| LB+RB | Emergency Stop |
| Button B (3s) | Magnetometer-Kalibrierung |

## Tools

| Tool | Funktion |
|------|----------|
| `./build_flash.sh ota` | BLE Firmware Update |
| `./start_dashboard.sh` | PyQt6 Telemetry GUI |
| `tools/analyze_telemetry.py` | FFT-Analyse für Gait-Frequenz |

## Dokumentation

Ausführliche technische Details: **[CLAUDE.md](CLAUDE.md)**

## Lizenz

Apache-2.0

---
*Stand: 2026-01-30*
