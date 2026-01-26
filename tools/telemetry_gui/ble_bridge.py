"""
BLE-Qt Bridge for Telemetry Receiver

Uses qasync for asyncio-Qt integration.
"""

import asyncio
import struct
from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal
from bleak import BleakClient, BleakScanner

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
THREAD_STATS_FORMAT = '<BBBBIHH'  # magic, version, thread_id, thread_count, timestamp, cpu_permille, stack_used
THREAD_STATS_SIZE = 12

# Thread ID names (matches firmware enum thread_id in telemetry.h)
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

# Navigation states (must match enum autonav_state in autonomous_nav.h)
NAV_STATES = {
    0: "DISABLED",
    1: "HEADING_HOLD",
    2: "TURNING",
    3: "SCANNING",
}


def parse_telemetry(data: bytes) -> Optional[dict]:
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
        'nav_state': fields[11],
        'nav_state_name': NAV_STATES.get(fields[11], f"UNKNOWN({fields[11]})"),
        'autonav_enabled': bool(fields[12] & 0x01),
        'motors_enabled': bool(fields[12] & 0x02),
        'target_heading': fields[13] / 10.0 if fields[13] != 0xFFFF else None,
        'raw_accel': (fields[14], fields[15], fields[16]),
        'raw_mag': (fields[17], fields[18], fields[19]),
    }


def parse_thread_stats(data: bytes) -> Optional[dict]:
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
        'thread_name': THREAD_NAMES.get(fields[2], f"thread_{fields[2]}"),
        'thread_count': fields[3],
        'timestamp_ms': fields[4],
        'cpu_percent': fields[5] / 10.0,  # permille to percent
        'stack_used': fields[6],
    }


class BLEReceiver(QObject):
    """BLE receiver with Qt signals for telemetry packets."""

    # Signals
    packet_received = pyqtSignal(dict)          # Main telemetry packet
    thread_stats_received = pyqtSignal(dict)    # Thread stats packet
    connection_changed = pyqtSignal(bool, str)  # connected, device_name
    scan_progress = pyqtSignal(str)             # status message
    error_occurred = pyqtSignal(str)            # error message

    # Reconnect settings
    RECONNECT_DELAY = 2.0  # seconds before reconnect attempt
    SCAN_TIMEOUT = 10.0    # seconds for BLE scan

    def __init__(self, address: Optional[str] = None, auto_reconnect: bool = True):
        super().__init__()
        self.address = address
        self.auto_reconnect = auto_reconnect
        self.client: Optional[BleakClient] = None
        self.buffer = bytearray()
        self.packet_count = 0
        self.last_timestamp = 0
        self.current_rate = 0.0
        self._running = False
        self._device_name = ""
        self._last_device_address: Optional[str] = None

    def _handle_notification(self, sender, data: bytes):
        """Handle incoming BLE notification."""
        self.buffer.extend(data)

        # Process packets - look for magic bytes
        while len(self.buffer) >= THREAD_STATS_SIZE:  # Minimum packet size
            # Find any magic byte
            magic_ab = -1
            magic_ac = -1
            try:
                magic_ab = self.buffer.index(TELEMETRY_MAGIC)
            except ValueError:
                pass
            try:
                magic_ac = self.buffer.index(THREAD_STATS_MAGIC)
            except ValueError:
                pass

            # No magic bytes found
            if magic_ab < 0 and magic_ac < 0:
                self.buffer.clear()
                break

            # Find earliest magic byte
            if magic_ab >= 0 and (magic_ac < 0 or magic_ab <= magic_ac):
                # Telemetry packet (0xAB)
                if magic_ab > 0:
                    del self.buffer[:magic_ab]

                if len(self.buffer) < TELEMETRY_SIZE:
                    break

                packet_data = bytes(self.buffer[:TELEMETRY_SIZE])
                del self.buffer[:TELEMETRY_SIZE]

                pkt = parse_telemetry(packet_data)
                if pkt:
                    self.packet_count += 1

                    # Calculate rate
                    if self.last_timestamp > 0:
                        dt = pkt['timestamp_ms'] - self.last_timestamp
                        if dt > 0:
                            self.current_rate = 1000.0 / dt
                    self.last_timestamp = pkt['timestamp_ms']

                    # Add rate to packet
                    pkt['rate_hz'] = self.current_rate
                    pkt['packet_count'] = self.packet_count

                    # Emit signal
                    self.packet_received.emit(pkt)

            else:
                # Thread stats packet (0xAC)
                if magic_ac > 0:
                    del self.buffer[:magic_ac]

                if len(self.buffer) < THREAD_STATS_SIZE:
                    break

                packet_data = bytes(self.buffer[:THREAD_STATS_SIZE])
                del self.buffer[:THREAD_STATS_SIZE]

                pkt = parse_thread_stats(packet_data)
                if pkt:
                    self.thread_stats_received.emit(pkt)

    async def _find_robot(self) -> Optional[object]:
        """Scan for the robot."""
        self.scan_progress.emit("Scanning for Ozzy...")
        devices = await BleakScanner.discover(timeout=10.0)

        for device in devices:
            if device.name and "Ozzy" in device.name:
                self.scan_progress.emit(f"Found: {device.name}")
                return device

        for device in devices:
            if device.name and "IMU" in device.name:
                self.scan_progress.emit(f"Found: {device.name}")
                return device

        return None

    async def connect(self):
        """Connect to the robot with auto-reconnect."""
        self._running = True

        while self._running:
            try:
                # Determine device address
                if self.address:
                    self.scan_progress.emit(f"Connecting to {self.address}...")
                    device_address = self.address
                    self._device_name = self.address
                elif self._last_device_address and self.auto_reconnect:
                    # Try last known device first
                    self.scan_progress.emit(f"Reconnecting to {self._device_name}...")
                    device_address = self._last_device_address
                else:
                    device = await self._find_robot()
                    if not device:
                        if self.auto_reconnect and self._running:
                            self.scan_progress.emit(f"Robot not found, retrying in {self.RECONNECT_DELAY}s...")
                            await asyncio.sleep(self.RECONNECT_DELAY)
                            continue
                        else:
                            self.error_occurred.emit("Robot not found!")
                            return
                    device_address = device.address
                    self._device_name = device.name or device.address

                # Remember for reconnect
                self._last_device_address = device_address

                # Connect
                self.client = BleakClient(device_address)
                await self.client.connect()

                if not self.client.is_connected:
                    raise Exception("Connection failed")

                self.connection_changed.emit(True, self._device_name)

                # Reset buffer on new connection
                self.buffer.clear()

                # Start notifications
                await self.client.start_notify(NUS_TX_UUID, self._handle_notification)

                # Keep running until disconnected
                while self._running and self.client.is_connected:
                    await asyncio.sleep(0.1)

                # Clean disconnect
                if self.client.is_connected:
                    await self.client.stop_notify(NUS_TX_UUID)
                    await self.client.disconnect()

            except Exception as e:
                self.error_occurred.emit(str(e))

            finally:
                self.connection_changed.emit(False, "")

            # Auto-reconnect logic
            if self._running and self.auto_reconnect:
                self.scan_progress.emit(f"Disconnected. Reconnecting in {self.RECONNECT_DELAY}s...")
                await asyncio.sleep(self.RECONNECT_DELAY)
            else:
                break

    async def disconnect(self):
        """Disconnect from the robot and stop auto-reconnect."""
        self._running = False
        self.auto_reconnect = False  # Prevent reconnect on manual disconnect
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(NUS_TX_UUID)
                await self.client.disconnect()
            except Exception:
                pass
