#!/usr/bin/env python3
"""
BLE Telemetry Receiver for Kosmos Proxi Robot

Receives and parses binary telemetry packets from the robot via BLE NUS.
Connect to the robot BEFORE pairing the Xbox controller.

Usage:
    python telemetry_receiver.py

Requirements:
    pip install bleak

Protocol:
    - Magic byte: 0xAB
    - 36-byte packets at ~20Hz
    - Little-endian (ARM native)
"""

import asyncio
import struct
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    exit(1)

# BLE UUIDs for Nordic UART Service (NUS)
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write to robot
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Receive from robot

# Telemetry packet format (36 bytes, little-endian)
# <BBIhhhhhHHbbBBHhhhhhh
#  B  - magic (0xAB)
#  B  - version (0x01)
#  I  - timestamp_ms
#  h  - roll_x10
#  h  - pitch_x10
#  h  - heading_x10 (actually uint16, but h works for unpacking)
#  h  - yaw_rate_x10
#  H  - ir_left_mm
#  H  - ir_right_mm
#  b  - motor_linear
#  b  - motor_angular
#  B  - nav_state
#  B  - flags
#  H  - target_heading_x10
#  h  - raw_ax
#  h  - raw_ay
#  h  - raw_az
#  h  - raw_mx
#  h  - raw_my
#  h  - raw_mz

TELEMETRY_MAGIC = 0xAB
# Format: magic, version, timestamp, roll, pitch, heading, yaw_rate, ir_l, ir_r,
#         motor_lin, motor_ang, nav_state, flags, target_hdg, ax, ay, az, mx, my, mz
TELEMETRY_FORMAT = '<BBIhhHhHHbbBBHhhhhhh'  # 36 bytes
TELEMETRY_SIZE = 36

# Navigation states
NAV_STATES = {
    0: "DISABLED",
    1: "HEADING_HOLD",
    2: "TURNING",
    3: "BACKING_UP",
}


def parse_telemetry(data: bytes) -> dict | None:
    """Parse a telemetry packet from raw bytes."""
    if len(data) != TELEMETRY_SIZE:
        return None

    if data[0] != TELEMETRY_MAGIC:
        return None

    fields = struct.unpack(TELEMETRY_FORMAT, data)

    return {
        'magic': fields[0],
        'version': fields[1],
        'timestamp_ms': fields[2],
        'roll': fields[3] / 10.0,
        'pitch': fields[4] / 10.0,
        'heading': fields[5] / 10.0,
        'yaw_rate': fields[6] / 10.0,
        'ir_left_mm': fields[7],
        'ir_right_mm': fields[8],
        'motor_linear': fields[9],
        'motor_angular': fields[10],
        'nav_state': NAV_STATES.get(fields[11], f"UNKNOWN({fields[11]})"),
        'autonav_enabled': bool(fields[12] & 0x01),
        'motors_enabled': bool(fields[12] & 0x02),
        'target_heading': fields[13] / 10.0 if fields[13] != 0xFFFF else None,
        'raw_accel': (fields[14], fields[15], fields[16]),
        'raw_mag': (fields[17], fields[18], fields[19]),
    }


def format_telemetry(pkt: dict) -> str:
    """Format telemetry packet for display."""
    lines = [
        f"[{pkt['timestamp_ms']:8d}ms] Roll:{pkt['roll']:6.1f}° Pitch:{pkt['pitch']:6.1f}° Hdg:{pkt['heading']:5.1f}°",
        f"  Yaw:{pkt['yaw_rate']:5.1f}°/s  IR L:{pkt['ir_left_mm']:4d}mm R:{pkt['ir_right_mm']:4d}mm",
        f"  Motor: lin={pkt['motor_linear']:4d} ang={pkt['motor_angular']:4d}",
        f"  Nav: {pkt['nav_state']} {'AUTO' if pkt['autonav_enabled'] else 'MANUAL'} {'MTR_ON' if pkt['motors_enabled'] else 'MTR_OFF'}",
    ]
    if pkt['target_heading'] is not None:
        lines.append(f"  Target: {pkt['target_heading']:.1f}°")
    return '\n'.join(lines)


class TelemetryReceiver:
    def __init__(self):
        self.buffer = bytearray()
        self.packet_count = 0
        self.last_timestamp = 0

    def handle_notification(self, sender, data: bytes):
        """Handle incoming BLE notification."""
        # Add to buffer
        self.buffer.extend(data)

        # Process complete packets
        while len(self.buffer) >= TELEMETRY_SIZE:
            # Look for magic byte
            try:
                start = self.buffer.index(TELEMETRY_MAGIC)
                if start > 0:
                    # Discard bytes before magic
                    del self.buffer[:start]
            except ValueError:
                # No magic byte found, discard all
                self.buffer.clear()
                break

            if len(self.buffer) < TELEMETRY_SIZE:
                break

            # Extract packet
            packet_data = bytes(self.buffer[:TELEMETRY_SIZE])
            del self.buffer[:TELEMETRY_SIZE]

            # Parse and display
            pkt = parse_telemetry(packet_data)
            if pkt:
                self.packet_count += 1

                # Calculate actual rate
                if self.last_timestamp > 0:
                    dt = pkt['timestamp_ms'] - self.last_timestamp
                    rate = 1000.0 / dt if dt > 0 else 0
                else:
                    rate = 0
                self.last_timestamp = pkt['timestamp_ms']

                # Clear screen and print (for nice updating display)
                print("\033[2J\033[H", end="")  # ANSI clear screen
                print(f"=== Kosmos Proxi Telemetry ({rate:.1f} Hz) ===")
                print(format_telemetry(pkt))
                print(f"\nPackets: {self.packet_count}")
                print("\nPress Ctrl+C to exit")


async def find_robot():
    """Scan for the robot (named 'Ozzy')."""
    print("Scanning for Ozzy...")
    devices = await BleakScanner.discover(timeout=10.0)

    # Show all discovered devices for debugging
    print(f"\nFound {len(devices)} BLE devices:")
    for device in devices:
        name = device.name or "(no name)"
        print(f"  - {name}: {device.address}")

    print()

    for device in devices:
        if device.name and "Ozzy" in device.name:
            print(f"Found robot: {device.name} ({device.address})")
            return device

    # Also try IMU-Sensor (legacy name)
    for device in devices:
        if device.name and "IMU" in device.name:
            print(f"Found robot: {device.name} ({device.address})")
            return device

    return None


async def main():
    print("Kosmos Proxi Telemetry Receiver")
    print("=" * 40)

    device = await find_robot()
    if not device:
        print("Robot not found! Make sure it's powered on and advertising.")
        print("Try connecting BEFORE pairing the Xbox controller.")
        return

    receiver = TelemetryReceiver()

    async with BleakClient(device.address) as client:
        print(f"Connected to {device.name}")

        # Enable notifications on NUS TX characteristic
        await client.start_notify(NUS_TX_UUID, receiver.handle_notification)
        print("Receiving telemetry... (press Ctrl+C to exit)")

        # Keep running
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nDisconnecting...")

        await client.stop_notify(NUS_TX_UUID)


if __name__ == "__main__":
    asyncio.run(main())
