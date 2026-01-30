# Kosmos Proxi Roboter Dokumentation

## Übersicht

Der Kosmos Proxi (Art.Nr. 620585) ist ein **Hexapod** (6-Bein-Roboter), der mit dem BBC micro:bit gesteuert wird.

- **Produktseite**: https://www.kosmos.de/de/proxi_1620585_4002051620585
- **Anleitung**: https://www.manualslib.de/manual/1525570/Kosmos-Proxi.html

---

## Hardware-Komponenten

### Proxi Zusatzplatine

| Komponente | Funktion |
|------------|----------|
| Edge Connector | Aufnahme für micro:bit |
| IR-Sensoren (2×) | Hinderniserkennung |
| Motor-Treiber | H-Brücke für 2 Motoren |
| Batteriefach-Anschluss | 4× AA (6V) |

---

## Pin-Zuordnung (Verifiziert 2026-01-25)

### Motor-Pins (H-Brücke)

| micro:bit Pin | nRF GPIO | Funktion |
|---------------|----------|----------|
| P13 | P0.17 | Geh-Motor: Vorwärts |
| P14 | P0.01 | Geh-Motor: Rückwärts |
| P15 | P0.13 | Dreh-Motor: Links |
| P16 | P1.02 | Dreh-Motor: Rechts |

**H-Brücken-Logik:**
- Vorwärts: P13=PWM, P14=LOW
- Rückwärts: P13=LOW, P14=PWM
- **Niemals beide HIGH!** (Kurzschluss)

### IR-Sensor-Pins

| micro:bit Pin | nRF GPIO | Funktion |
|---------------|----------|----------|
| P0 | P0.02 (AIN0) | IR Sensor Links |
| P1 | P0.03 (AIN1) | IR Sensor Rechts |
| P12 | P0.12 | IR LED Enable (Active-HIGH) |

---

## IR-Sensor Signalverarbeitung

### Messprinzip

Die IR-Sensoren messen reflektiertes Infrarot-Licht zur Hinderniserkennung.

```
┌─────────────────────────────────────────────────────────────┐
│                    IR-SIGNALKETTE                            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. DIFFERENZMESSUNG (Fremdlicht-Eliminierung)              │
│     ├─ ADC mit IR-LEDs AUS → ambient                        │
│     ├─ ADC mit IR-LEDs AN  → reflection                     │
│     └─ ir_raw = reflection - ambient                        │
│                                                              │
│  2. LOW-PASS FILTER (10Hz Cutoff, IIR 1. Ordnung)           │
│     └─ ir_filt = 0.77×ir_raw + 0.23×ir_filt_prev            │
│                                                              │
│  3. BIAS-KOMPENSATION (Startup-Kalibrierung)                │
│     └─ ir = ir_filt - bias  (bei Start 20 Samples mitteln)  │
│                                                              │
│  4. ADC → DISTANZ (Lookup-Tabelle + Interpolation)          │
│     │   ADC    │  Distanz                                   │
│     │   100    │  550 mm                                    │
│     │   350    │  350 mm                                    │
│     │  1050    │  250 mm                                    │
│     │  3000    │  180 mm                                    │
│     │  4000    │  100 mm                                    │
│                                                              │
│  5. 1D KALMAN FILTER (Glättung)                             │
│     └─ Q=100mm², R=400mm² → stabile Distanzschätzung        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### ADC-Konfiguration

| Parameter | Wert |
|-----------|------|
| Auflösung | 12-bit (0-4095) |
| Gain | 4× (0-0.15V Bereich) |
| Referenz | Internal 0.6V |
| Oversampling | 16× Hardware |

### Kalman-Filter Parameter

```c
#define KALMAN_Q_DISTANCE   100.0f   /* Prozessrauschen (mm²) */
#define KALMAN_R_DISTANCE   400.0f   /* Messrauschen (mm²) */
```

**Modell:** Konstante Position (Hindernisse bewegen sich langsam)
```
d[k+1] = d[k]           /* Prädiktion */
z = raw_to_mm(adc)      /* Messung */
```

---

## Stromversorgung

- **4× AA Batterien** (6V) im Batteriefach
- USB über micro:bit nur für Programmierung (nicht für Motoren)

---

## Erweiterte Programmierung

Dieses Projekt nutzt **Zephyr RTOS** statt der Standard-MakeCode-Programmierung:
- Preemptive Threads für Motor, Sensoren, BLE
- BLE Dual-Role (Controller + Debug/OTA)
- Kalman-Filter für Sensorfusion
- OTA-Updates via Bluetooth
