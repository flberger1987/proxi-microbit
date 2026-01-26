#!/usr/bin/env python3
"""
BLE Telemetry Receiver for Kosmos Proxi Robot

Receives and parses binary telemetry packets from the robot via BLE NUS.
Connect to the robot BEFORE pairing the Xbox controller.

Usage:
    python telemetry_receiver.py          # Console mode
    python telemetry_receiver.py --gui    # GUI mode
    python telemetry_receiver.py -g       # GUI mode (short)

Requirements:
    pip install bleak                     # For console mode
    pip install -r requirements.txt       # For GUI mode (adds PyQt6, pyqtgraph, qasync)

Protocol:
    - Magic byte: 0xAB
    - 36-byte packets at ~20Hz
    - Little-endian (ARM native)
"""

import argparse
import asyncio
import struct
import os
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
TELEMETRY_MAGIC = 0xAB
TELEMETRY_FORMAT = '<BBIhhHhHHbbBBHhhhhhh'
TELEMETRY_SIZE = 36

# Thread stats packet format (12 bytes, little-endian)
THREAD_STATS_MAGIC = 0xAC
THREAD_STATS_FORMAT = '<BBBBIHH'
THREAD_STATS_SIZE = 12

# Thread IDs
THREAD_NAMES = {
    0: "main",
    1: "motor",
    2: "sensor",
    3: "ble_ctrl",
    4: "telemetry",
    5: "audio",
    6: "ir_sensors",
    7: "autonav",
    8: "idle",
}

# Navigation states
# Navigation states (must match enum autonav_state in autonomous_nav.h)
NAV_STATES = {
    0: "DISABLED",
    1: "HEADING_HOLD",
    2: "TURNING",
    3: "SCANNING",
}


def parse_telemetry(data: bytes) -> dict | None:
    """Parse a telemetry packet from raw bytes."""
    if len(data) != TELEMETRY_SIZE:
        return None

    if data[0] != TELEMETRY_MAGIC:
        return None

    fields = struct.unpack(TELEMETRY_FORMAT, data)

    return {
        'type': 'telemetry',
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


def parse_thread_stats(data: bytes) -> dict | None:
    """Parse a thread stats packet from raw bytes."""
    if len(data) != THREAD_STATS_SIZE:
        return None

    if data[0] != THREAD_STATS_MAGIC:
        return None

    fields = struct.unpack(THREAD_STATS_FORMAT, data)

    return {
        'type': 'thread_stats',
        'magic': fields[0],
        'version': fields[1],
        'thread_id': fields[2],
        'thread_name': THREAD_NAMES.get(fields[2], f"unknown({fields[2]})"),
        'thread_count': fields[3],
        'timestamp_ms': fields[4],
        'cpu_permille': fields[5],        # 0-1000 = 0.0%-100.0%
        'cpu_percent': fields[5] / 10.0,  # Convenience field
        'stack_used': fields[6],
    }


def format_telemetry(pkt: dict) -> str:
    """Format telemetry packet for display."""
    lines = [
        f"[{pkt['timestamp_ms']:8d}ms] Roll:{pkt['roll']:6.1f}\u00b0 Pitch:{pkt['pitch']:6.1f}\u00b0 Hdg:{pkt['heading']:5.1f}\u00b0",
        f"  Yaw:{pkt['yaw_rate']:5.1f}\u00b0/s  IR L:{pkt['ir_left_mm']:4d}mm R:{pkt['ir_right_mm']:4d}mm",
        f"  Motor: lin={pkt['motor_linear']:4d} ang={pkt['motor_angular']:4d}",
        f"  Nav: {pkt['nav_state']} {'AUTO' if pkt['autonav_enabled'] else 'MANUAL'} {'MTR_ON' if pkt['motors_enabled'] else 'MTR_OFF'}",
    ]
    if pkt['target_heading'] is not None:
        lines.append(f"  Target: {pkt['target_heading']:.1f}\u00b0")
    return '\n'.join(lines)


class TelemetryReceiver:
    def __init__(self, log_file: str | None = None):
        self.buffer = bytearray()
        self.packet_count = 0
        self.thread_packet_count = 0
        self.last_timestamp = 0
        self.last_telemetry = None
        self.thread_stats = {}  # thread_id -> latest stats
        self.rate = 0.0
        self.log_file = None
        self.log_path = log_file

        # Open log file if specified
        if log_file:
            self._init_log_file(log_file)

    def _init_log_file(self, log_file: str):
        """Initialize CSV log file with header."""
        self.log_file = open(log_file, 'w')
        # Write CSV header
        self.log_file.write(
            "timestamp_ms,roll,pitch,heading,target_heading,yaw_rate,"
            "ir_left_mm,ir_right_mm,motor_linear,motor_angular,"
            "nav_state,autonav_enabled,motors_enabled,"
            "raw_ax,raw_ay,raw_az,raw_mx,raw_my,raw_mz\n"
        )
        self.log_file.flush()
        print(f"Logging to: {log_file}")

    def _log_packet(self, pkt: dict):
        """Write telemetry packet to log file."""
        if not self.log_file:
            return

        target = pkt['target_heading'] if pkt['target_heading'] is not None else -1
        raw_a = pkt['raw_accel']
        raw_m = pkt['raw_mag']

        line = (
            f"{pkt['timestamp_ms']},"
            f"{pkt['roll']:.1f},{pkt['pitch']:.1f},{pkt['heading']:.1f},{target:.1f},"
            f"{pkt['yaw_rate']:.1f},"
            f"{pkt['ir_left_mm']},{pkt['ir_right_mm']},"
            f"{pkt['motor_linear']},{pkt['motor_angular']},"
            f"{pkt['nav_state']},{int(pkt['autonav_enabled'])},{int(pkt['motors_enabled'])},"
            f"{raw_a[0]},{raw_a[1]},{raw_a[2]},"
            f"{raw_m[0]},{raw_m[1]},{raw_m[2]}\n"
        )
        self.log_file.write(line)

        # Flush every 20 packets (~1 second)
        if self.packet_count % 20 == 0:
            self.log_file.flush()

    def close(self):
        """Close log file."""
        if self.log_file:
            self.log_file.close()
            print(f"Log saved to: {self.log_path}")

    def handle_notification(self, sender, data: bytes):
        """Handle incoming BLE notification."""
        # Add to buffer
        self.buffer.extend(data)

        # Process all complete packets
        while len(self.buffer) >= THREAD_STATS_SIZE:  # Smallest packet
            # Check first byte for magic
            magic = self.buffer[0]

            if magic == TELEMETRY_MAGIC:
                if len(self.buffer) < TELEMETRY_SIZE:
                    break  # Wait for more data

                packet_data = bytes(self.buffer[:TELEMETRY_SIZE])
                del self.buffer[:TELEMETRY_SIZE]

                pkt = parse_telemetry(packet_data)
                if pkt:
                    self.packet_count += 1
                    self.last_telemetry = pkt

                    # Calculate actual rate
                    if self.last_timestamp > 0:
                        dt = pkt['timestamp_ms'] - self.last_timestamp
                        self.rate = 1000.0 / dt if dt > 0 else 0
                    self.last_timestamp = pkt['timestamp_ms']

                    # Log to file
                    self._log_packet(pkt)

                    self._update_display()

            elif magic == THREAD_STATS_MAGIC:
                if len(self.buffer) < THREAD_STATS_SIZE:
                    break  # Wait for more data

                packet_data = bytes(self.buffer[:THREAD_STATS_SIZE])
                del self.buffer[:THREAD_STATS_SIZE]

                pkt = parse_thread_stats(packet_data)
                if pkt:
                    self.thread_packet_count += 1
                    self.thread_stats[pkt['thread_id']] = pkt
                    self._update_display()

            else:
                # Unknown magic, skip one byte
                del self.buffer[:1]

    def _update_display(self):
        """Update console display."""
        # Clear screen and print
        print("\033[2J\033[H", end="")  # ANSI clear screen
        print(f"=== Kosmos Proxi Telemetry ({self.rate:.1f} Hz) ===")

        if self.last_telemetry:
            print(format_telemetry(self.last_telemetry))

        # Thread stats section
        if self.thread_stats:
            print("\n--- Thread Stats ---")
            for tid in sorted(self.thread_stats.keys()):
                ts = self.thread_stats[tid]
                print(f"  {ts['thread_name']:12s}: CPU {ts['cpu_percent']:5.1f}%  Stack {ts['stack_used']:4d}B")

        print(f"\nPackets: {self.packet_count} tele, {self.thread_packet_count} thread")
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


async def run_console(address: str | None = None, log_file: str | None = None):
    """Run in console mode."""
    print("Kosmos Proxi Telemetry Receiver")
    print("=" * 40)

    if address:
        device_address = address
        device_name = address
        print(f"Connecting to {address}...")
    else:
        device = await find_robot()
        if not device:
            print("Robot not found! Make sure it's powered on and advertising.")
            print("Try connecting BEFORE pairing the Xbox controller.")
            return
        device_address = device.address
        device_name = device.name

    receiver = TelemetryReceiver(log_file=log_file)

    try:
        async with BleakClient(device_address) as client:
            print(f"Connected to {device_name}")

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
    finally:
        receiver.close()


def run_gui(address: str | None = None):
    """Run in GUI mode."""
    try:
        from PyQt6.QtWidgets import QApplication
        import qasync
    except ImportError:
        print("GUI mode requires additional dependencies.")
        print("Please install: pip install -r requirements.txt")
        print("  (PyQt6, pyqtgraph, qasync)")
        exit(1)

    from telemetry_gui import TelemetryDashboard, BLEReceiver

    # Create Qt application
    app = QApplication([])
    app.setApplicationName("Kosmos Proxi Telemetry")

    # Create event loop with qasync
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)

    # Create BLE receiver
    receiver = BLEReceiver(address)

    # Create dashboard
    dashboard = TelemetryDashboard(receiver)
    dashboard.show()

    # Start BLE connection in background
    async def start_ble():
        await receiver.connect()

    asyncio.ensure_future(start_ble())

    # Run the event loop
    with loop:
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Kosmos Proxi Telemetry Receiver",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python telemetry_receiver.py              # Console mode with auto-scan
  python telemetry_receiver.py --gui        # GUI dashboard
  python telemetry_receiver.py -g           # GUI dashboard (short)
  python telemetry_receiver.py --address XX:XX:XX:XX:XX:XX  # Direct connect
        """
    )

    parser.add_argument(
        '-g', '--gui',
        action='store_true',
        help='Launch GUI dashboard instead of console output'
    )

    parser.add_argument(
        '-a', '--address',
        type=str,
        default=None,
        help='BLE device address (skip scanning)'
    )

    parser.add_argument(
        '-l', '--log',
        type=str,
        default=None,
        metavar='FILE',
        help='Log telemetry to CSV file (e.g., --log telemetry.csv)'
    )

    args = parser.parse_args()

    if args.gui:
        run_gui(args.address)
    else:
        asyncio.run(run_console(args.address, log_file=args.log))


if __name__ == "__main__":
    main()
