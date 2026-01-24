#!/usr/bin/env python3
"""
Controller Button Mapping Session
Hält den seriellen Port persistent offen und sammelt Tasteneingaben.
"""

import serial
import sys
import time
import json
from collections import defaultdict
from datetime import datetime

PORT = '/dev/cu.usbmodem11202'
BAUD = 115200
MAPPING_FILE = '/Users/fb/ProxiMicro/controller_mapping.json'
LOG_FILE = '/Users/fb/ProxiMicro/controller_raw.log'

def main():
    print(f"=== Controller Mapping Session ===")
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUD}")
    print(f"Log: {LOG_FILE}")
    print(f"Mapping: {MAPPING_FILE}")
    print()

    # Port öffnen OHNE DTR/RTS Reset
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = 0.1  # Kurzer Timeout für non-blocking read
    ser.dtr = False    # KEIN DTR - verhindert Reset!
    ser.rts = False    # KEIN RTS - verhindert Reset!

    try:
        ser.open()
        print(f"✓ Port geöffnet (DTR/RTS deaktiviert)")
        print()
        print("Empfange Daten... (Drücke Tasten am Controller)")
        print("-" * 60)

        # Mapping Dictionary
        button_events = defaultdict(list)

        with open(LOG_FILE, 'a') as log:
            log.write(f"\n=== Session Start: {datetime.now().isoformat()} ===\n")

            last_data = None
            repeat_count = 0

            while True:
                try:
                    line = ser.readline()
                    if line:
                        decoded = line.decode('utf-8', errors='ignore').strip()
                        if decoded:
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                            # Wiederholungen komprimieren
                            if decoded == last_data:
                                repeat_count += 1
                                # Nur alle 10 Wiederholungen anzeigen
                                if repeat_count % 10 == 0:
                                    print(f"[{timestamp}] (x{repeat_count}) {decoded}")
                            else:
                                if repeat_count > 1:
                                    print(f"  └─ Wiederholt {repeat_count}x")
                                repeat_count = 1
                                last_data = decoded
                                print(f"[{timestamp}] {decoded}")

                            # In Log schreiben
                            log.write(f"[{timestamp}] {decoded}\n")
                            log.flush()

                except KeyboardInterrupt:
                    print("\n\nSession beendet.")
                    break
                except Exception as e:
                    print(f"Fehler beim Lesen: {e}")
                    time.sleep(0.1)

    except serial.SerialException as e:
        print(f"✗ Fehler beim Öffnen des Ports: {e}")
        sys.exit(1)
    finally:
        if ser.is_open:
            ser.close()
            print("Port geschlossen.")

if __name__ == "__main__":
    main()
