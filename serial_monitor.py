#!/usr/bin/env python3
"""Serial monitor for micro:bit - non-blocking with timeout"""
import serial
import sys
import os

# Force unbuffered output
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

PORT = '/dev/cu.usbmodem11202'  # micro:bit serial port
BAUD = 115200

print(f"Connecting to {PORT} at {BAUD} baud...", flush=True)
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print("Connected. Reading output (Ctrl+C to stop):", flush=True)
    print("="*50, flush=True)
    while True:
        line = ser.readline()
        if line:
            try:
                decoded = line.decode('utf-8', errors='ignore')
                print(decoded, end='', flush=True)
            except Exception as e:
                print(f"[decode error: {e}]", flush=True)
except serial.SerialException as e:
    print(f"Serial error: {e}", file=sys.stderr, flush=True)
    sys.exit(1)
except KeyboardInterrupt:
    print("\nStopped.", flush=True)
finally:
    if 'ser' in dir() and ser.is_open:
        ser.close()
