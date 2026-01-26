# Kosmos Proxi Roboter Dokumentation

## Übersicht

Der Kosmos Proxi (Art.Nr. 620585) ist ein **Hexapod** (6-Bein-Roboter), der mit dem BBC micro:bit gesteuert wird. Er kann vorwärts/rückwärts gehen und sich um seine Achse drehen.

## Offizielle Quellen

- **Produktseite**: https://www.kosmos.de/de/proxi_1620585_4002051620585
- **Weitere Informationen**: https://www.kosmos.de/de/content/Microsites/Science/KOSMOS%20Proxi%20-%20Weitere%20Produktinformationen
- **Anleitung (ManualsLib)**: https://www.manualslib.de/manual/1525570/Kosmos-Proxi.html
- **Kundenservice**: service@kosmos.de

## Hardware-Komponenten

### micro:bit Platine (inkludiert)
- Programmierbarer Microcontroller mit eigenem Speicher
- Bluetooth
- Beschleunigungssensor
- Kompass
- 2 programmierbare Tasten
- 5x5-LED-Display (auch als Lichtsensor nutzbar)
- Temperatur-Sensor (Prozessor-basiert)
- Pins zur Ausgabe und Eingabe von Signalen

### Proxi Zusatzplatine
Die Proxi Zusatzplatine erweitert den micro:bit um roboterspezifische Funktionen:

| Komponente | Funktion |
|------------|----------|
| Edge Connector | Aufnahme für micro:bit |
| Infrarot-Sensoren | Hinderniserkennung, Linienfolgen |
| Buzzer | Akustische Ausgabe |
| Batteriefach-Anschluss | Stromversorgung |
| Motor-Treiber | Steuerung der Hexapod-Motoren |

## Schaltplan

**Hinweis**: Kein öffentlicher Schaltplan verfügbar. Pin-Belegung wurde durch Reverse Engineering ermittelt.

## Pin-Zuordnung (Verifiziert 2025-01-25)

Der Proxi verwendet **zwei Motoren** mit H-Brücken-Steuerung:

### Motor-Pins

| micro:bit Pin | nRF GPIO | Funktion |
|---------------|----------|----------|
| P13 | P0.17 | Geh-Motor: Vorwärts |
| P14 | P0.01 | Geh-Motor: Rückwärts |
| P15 | P0.13 | Dreh-Motor: Links |
| P16 | P1.02 | Dreh-Motor: Rechts |

### H-Brücken-Logik

**Geh-Motor (P13/P14):**
- Vorwärts: P13=HIGH, P14=LOW
- Rückwärts: P13=LOW, P14=HIGH
- Stopp: P13=LOW, P14=LOW

**Dreh-Motor (P15/P16):**
- Links drehen: P15=HIGH, P16=LOW
- Rechts drehen: P15=LOW, P16=HIGH
- Stopp: P15=LOW, P16=LOW

**WARNUNG**: Niemals beide Pins einer H-Brücke gleichzeitig HIGH setzen!

### IR-Sensor-Pins (Verifiziert 2025-01-25)

| micro:bit Pin | nRF GPIO | ADC | Funktion |
|---------------|----------|-----|----------|
| P0 | P0.02 | AIN0 | IR Sensor Links |
| P1 | P0.03 | AIN1 | IR Sensor Rechts |
| P12 | P0.12 | GPIO | IR LED Enable (Active-HIGH) |
| P19/20 | - | - | I2C (LSM303AGR) |

**Messprinzip:**
1. Messe ADC mit IR-LEDs AUS (Ambient-Licht)
2. Schalte IR-LEDs EIN (P12=HIGH)
3. Messe ADC mit IR-LEDs AN (Reflection)
4. Differenz = tatsächlicher IR-Wert (eliminiert Fremdlicht)

## Standard-Programmierung

Die offizielle Programmierung erfolgt über:
- **MakeCode** (Block-basiert): https://makecode.microbit.org/
- **MakeCode Python**
- **micro:bit Classroom**

## Erweiterte Programmierung

Für fortgeschrittene Anwendungen siehe:
- `docs/software/zephyr-rtos.md` - Zephyr RTOS Setup
- `docs/software/codal-sdk.md` - CODAL SDK für C++

## Stromversorgung

- 4x AA Batterien (6V) im Batteriefach
- Alternativ: USB über micro:bit (nur für Programmierung, nicht für Motoren)
