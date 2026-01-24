# Kosmos Proxi Roboter Dokumentation

## Übersicht

Der Kosmos Proxi (Art.Nr. 620585) ist ein Programmier-Roboter, der mit dem BBC micro:bit gesteuert wird.

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
| Motor-Treiber | Steuerung der Antriebsmotoren |

## Schaltplan

**Hinweis**: Es wurde kein öffentlich verfügbarer Schaltplan für die Proxi-Zusatzplatine gefunden.

Mögliche Quellen:
1. Direkter Kontakt mit KOSMOS Kundenservice (service@kosmos.de)
2. Reverse Engineering der Platine
3. micro:bit Community Foren

## Pin-Zuordnung (geschätzt)

Basierend auf typischen micro:bit Roboter-Erweiterungen:

| micro:bit Pin | Proxi Funktion (vermutet) |
|---------------|---------------------------|
| P0 | Motor Links PWM |
| P1 | Motor Rechts PWM |
| P2 | Buzzer |
| P8 | IR Sensor Links |
| P12 | IR Sensor Rechts |
| P13-15 | SPI (reserviert) |
| P19/20 | I2C (reserviert) |

**Achtung**: Diese Zuordnung muss durch Analyse der Platine verifiziert werden!

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
