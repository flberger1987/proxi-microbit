"""
Serial port reader for IMU orientation data.

Protocol:
  IMU,<timestamp_ms>,<roll>,<pitch>,<heading>\r\n
  CAL,START
  CAL,DONE,<ox>,<oy>,<oz>
"""

import serial
import serial.tools.list_ports
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Callable
import time


@dataclass
class OrientationData:
    """Orientation data from micro:bit"""
    roll: float
    pitch: float
    heading: float
    timestamp_ms: int


@dataclass
class CalibrationData:
    """Calibration status"""
    in_progress: bool
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0


def find_microbit_port() -> Optional[str]:
    """Find the micro:bit serial port on macOS."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # micro:bit shows up as usbmodem on macOS
        if 'usbmodem' in port.device.lower():
            return port.device
        # Also check for micro:bit in description
        if port.description and 'micro:bit' in port.description.lower():
            return port.device
    return None


class SerialReader:
    """
    Reads orientation data from micro:bit over serial port.
    Runs in a background thread and provides data via queue.
    """

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Data queues
        self.orientation_queue: queue.Queue[OrientationData] = queue.Queue(maxsize=100)
        self.calibration_callback: Optional[Callable[[CalibrationData], None]] = None

        # Latest data (for polling)
        self._latest_orientation: Optional[OrientationData] = None
        self._lock = threading.Lock()

    def set_calibration_callback(self, callback: Callable[[CalibrationData], None]):
        """Set callback for calibration events."""
        self.calibration_callback = callback

    def connect(self) -> bool:
        """Connect to the serial port."""
        if self.port is None:
            self.port = find_microbit_port()

        if self.port is None:
            print("ERROR: Could not find micro:bit serial port")
            print("Available ports:")
            for port in serial.tools.list_ports.comports():
                print(f"  {port.device}: {port.description}")
            return False

        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1.0
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Could not open serial port: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port."""
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from serial port")

    def start(self):
        """Start the reader thread."""
        if self.serial is None or not self.serial.is_open:
            if not self.connect():
                return False

        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        """Stop the reader thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

    def get_latest(self) -> Optional[OrientationData]:
        """Get the latest orientation data (non-blocking)."""
        with self._lock:
            return self._latest_orientation

    def get_orientation(self, timeout: float = 0.1) -> Optional[OrientationData]:
        """Get orientation data from queue (blocking with timeout)."""
        try:
            return self.orientation_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _parse_line(self, line: str):
        """Parse a line of data from the micro:bit."""
        line = line.strip()
        if not line:
            return

        parts = line.split(',')

        if parts[0] == 'IMU' and len(parts) == 5:
            try:
                data = OrientationData(
                    timestamp_ms=int(parts[1]),
                    roll=float(parts[2]),
                    pitch=float(parts[3]),
                    heading=float(parts[4])
                )

                # Update latest
                with self._lock:
                    self._latest_orientation = data

                # Add to queue (drop oldest if full)
                try:
                    self.orientation_queue.put_nowait(data)
                except queue.Full:
                    try:
                        self.orientation_queue.get_nowait()
                        self.orientation_queue.put_nowait(data)
                    except queue.Empty:
                        pass

            except (ValueError, IndexError) as e:
                print(f"Parse error: {e} in line: {line}")

        elif parts[0] == 'CAL':
            if len(parts) >= 2:
                if parts[1] == 'START':
                    print("Calibration started - rotate device in all directions")
                    if self.calibration_callback:
                        self.calibration_callback(CalibrationData(in_progress=True))

                elif parts[1] == 'DONE' and len(parts) == 5:
                    try:
                        cal = CalibrationData(
                            in_progress=False,
                            offset_x=float(parts[2]),
                            offset_y=float(parts[3]),
                            offset_z=float(parts[4])
                        )
                        print(f"Calibration done: offset=({cal.offset_x:.3f}, {cal.offset_y:.3f}, {cal.offset_z:.3f})")
                        if self.calibration_callback:
                            self.calibration_callback(cal)
                    except (ValueError, IndexError) as e:
                        print(f"Calibration parse error: {e}")

    def _read_loop(self):
        """Background thread that reads from serial port."""
        buffer = ""

        while self.running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._parse_line(line)
                else:
                    time.sleep(0.01)

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.running = False
                break
            except Exception as e:
                print(f"Reader error: {e}")
                time.sleep(0.1)


if __name__ == '__main__':
    # Test the serial reader
    reader = SerialReader()

    def on_calibration(cal: CalibrationData):
        print(f"Calibration event: {cal}")

    reader.set_calibration_callback(on_calibration)

    if reader.start():
        print("Reading orientation data (Ctrl+C to stop)...")
        try:
            while True:
                data = reader.get_orientation(timeout=0.5)
                if data:
                    print(f"Roll: {data.roll:7.1f}  Pitch: {data.pitch:7.1f}  Heading: {data.heading:7.1f}")
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            reader.disconnect()
