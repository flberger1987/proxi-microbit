#!/usr/bin/env python3
"""
IR Distance Sensor Plotter - Live visualization with Kalman filtering

Parses serial data:
  IR,<timestamp_ms>,<left_mm>,<right_mm>,<left_raw>,<right_raw>,<left_mm_raw>,<right_mm_raw>

Controls:
  SPACE     - Pause/Resume
  C         - Clear data
  S         - Save current data to CSV
  M         - Measure mode (click two points to get time delta)
  D         - Toggle debug output on device

Usage:
    python3 ir_plot.py [port]

Requirements:
    pip install pyserial matplotlib
"""

import sys
import glob
import time
import serial
import csv
from collections import deque
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configuration
BUFFER_SIZE = 500       # Number of data points to display
UPDATE_INTERVAL = 50    # Graph update interval in ms
BAUD_RATE = 115200


def find_serial_port():
    """Auto-detect micro:bit serial port."""
    patterns = [
        '/dev/cu.usbmodem*',      # macOS
        '/dev/ttyACM*',           # Linux
        'COM*',                   # Windows
    ]
    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]
    return None


class IRPlotter:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.paused = False
        self.measure_mode = False
        self.measure_points = []

        # IR data buffers (Kalman filtered)
        self.timestamps = deque(maxlen=BUFFER_SIZE)
        self.left_mm = deque(maxlen=BUFFER_SIZE)
        self.right_mm = deque(maxlen=BUFFER_SIZE)

        # Raw data buffers
        self.left_raw = deque(maxlen=BUFFER_SIZE)
        self.right_raw = deque(maxlen=BUFFER_SIZE)

        # Raw mm (before Kalman)
        self.left_mm_raw = deque(maxlen=BUFFER_SIZE)
        self.right_mm_raw = deque(maxlen=BUFFER_SIZE)

        # Statistics
        self.ir_count = 0
        self.start_time = time.time()
        self.raw_lines = []  # Store raw data for CSV export

        # Setup plot with 3 subplots
        self.fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        self.ax1, self.ax2, self.ax3 = axes
        self.fig.suptitle('IR Distance Sensors (Kalman Filtered)  [SPACE=Pause, C=Clear, S=Save, D=Toggle Debug]', fontsize=12)

        # Distance subplot (Kalman filtered)
        self.ax1.set_ylabel('Distance (mm)')
        self.ax1.set_ylim(0, 600)
        self.ax1.set_xlim(0, BUFFER_SIZE)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axhline(y=400, color='orange', linestyle='--', alpha=0.5, label='Beep Start')
        self.ax1.axhline(y=100, color='red', linestyle='--', alpha=0.5, label='Beep Max')
        self.line_left_mm, = self.ax1.plot([], [], 'b-', label='Left (filtered)', linewidth=2)
        self.line_right_mm, = self.ax1.plot([], [], 'r-', label='Right (filtered)', linewidth=2)
        self.line_left_mm_raw, = self.ax1.plot([], [], 'b--', label='Left (raw)', linewidth=1, alpha=0.4)
        self.line_right_mm_raw, = self.ax1.plot([], [], 'r--', label='Right (raw)', linewidth=1, alpha=0.4)
        self.ax1.legend(loc='upper right')
        self.ax1.set_title('Kalman Filtered Distance')

        # Raw ADC values subplot
        self.ax2.set_ylabel('ADC Value')
        self.ax2.set_ylim(0, 4096)
        self.ax2.set_xlim(0, BUFFER_SIZE)
        self.ax2.grid(True, alpha=0.3)
        self.line_left_raw, = self.ax2.plot([], [], 'b-', label='Left Raw', linewidth=1.5)
        self.line_right_raw, = self.ax2.plot([], [], 'r-', label='Right Raw', linewidth=1.5)
        self.ax2.legend(loc='upper right')
        self.ax2.set_title('Raw IR Sensor Values (bias-corrected)')

        # Kalman gain / innovation subplot
        self.ax3.set_ylabel('Distance Diff (mm)')
        self.ax3.set_xlabel('Sample')
        self.ax3.set_ylim(-100, 100)
        self.ax3.set_xlim(0, BUFFER_SIZE)
        self.ax3.grid(True, alpha=0.3)
        self.ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.line_left_diff, = self.ax3.plot([], [], 'b-', label='Left (filt-raw)', linewidth=1.5)
        self.line_right_diff, = self.ax3.plot([], [], 'r-', label='Right (filt-raw)', linewidth=1.5)
        self.ax3.legend(loc='upper right')
        self.ax3.set_title('Kalman Innovation (filtered - raw)')

        # Status text
        self.status_text = self.fig.text(0.02, 0.02, '', fontsize=10, family='monospace')

        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        plt.tight_layout()

    def connect(self):
        """Connect to serial port."""
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = BAUD_RATE
            self.ser.timeout = 0.1
            self.ser.dtr = False  # Don't reset micro:bit
            self.ser.rts = False
            self.ser.open()
            print(f"Connected to {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def parse_line(self, line):
        """Parse a single line of IR data."""
        line = line.strip()
        if not line:
            return

        self.raw_lines.append(line)

        # Parse IR data: IR,<ts>,<left_mm>,<right_mm>,<left_raw>,<right_raw>,<left_mm_raw>,<right_mm_raw>
        if line.startswith('IR,'):
            try:
                parts = line.split(',')
                if len(parts) >= 7:
                    ts = int(parts[1])
                    left_mm = float(parts[2])
                    right_mm = float(parts[3])
                    left_raw = int(parts[4])
                    right_raw = int(parts[5])
                    left_mm_raw = float(parts[6])
                    right_mm_raw = float(parts[7]) if len(parts) > 7 else left_mm_raw

                    if not self.paused:
                        self.timestamps.append(ts)
                        self.left_mm.append(left_mm)
                        self.right_mm.append(right_mm)
                        self.left_raw.append(left_raw)
                        self.right_raw.append(right_raw)
                        self.left_mm_raw.append(left_mm_raw)
                        self.right_mm_raw.append(right_mm_raw)
                        self.ir_count += 1
            except (ValueError, IndexError) as e:
                pass  # Ignore malformed lines
        else:
            # Print other messages
            print(line)

    def read_serial(self):
        """Read and parse serial data."""
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                self.parse_line(line)
        except Exception as e:
            print(f"Serial read error: {e}")

    def update_plot(self, frame):
        """Animation update function."""
        self.read_serial()

        # Calculate indices for x-axis
        n = len(self.timestamps)
        x = list(range(n))

        # Update distance plot (Kalman filtered + raw)
        self.line_left_mm.set_data(x, list(self.left_mm))
        self.line_right_mm.set_data(x, list(self.right_mm))
        self.line_left_mm_raw.set_data(x, list(self.left_mm_raw))
        self.line_right_mm_raw.set_data(x, list(self.right_mm_raw))

        # Update raw ADC plot
        self.line_left_raw.set_data(x, list(self.left_raw))
        self.line_right_raw.set_data(x, list(self.right_raw))

        # Calculate and update innovation (filtered - raw)
        left_diff = [f - r for f, r in zip(self.left_mm, self.left_mm_raw)]
        right_diff = [f - r for f, r in zip(self.right_mm, self.right_mm_raw)]
        self.line_left_diff.set_data(x, left_diff)
        self.line_right_diff.set_data(x, right_diff)

        # Update status
        elapsed = time.time() - self.start_time
        hz = self.ir_count / elapsed if elapsed > 0 else 0

        left_mm_curr = self.left_mm[-1] if self.left_mm else 0
        right_mm_curr = self.right_mm[-1] if self.right_mm else 0
        left_raw_curr = self.left_raw[-1] if self.left_raw else 0
        right_raw_curr = self.right_raw[-1] if self.right_raw else 0

        status = f"Samples: {self.ir_count}  Rate: {hz:.1f} Hz  "
        status += f"Left: {left_mm_curr:.0f}mm (raw:{left_raw_curr})  "
        status += f"Right: {right_mm_curr:.0f}mm (raw:{right_raw_curr})  "
        if self.paused:
            status += "[PAUSED]"
        if self.measure_mode:
            status += "[MEASURE MODE]"

        self.status_text.set_text(status)

        return (self.line_left_mm, self.line_right_mm, self.line_left_mm_raw,
                self.line_right_mm_raw, self.line_left_raw, self.line_right_raw,
                self.line_left_diff, self.line_right_diff, self.status_text)

    def on_key(self, event):
        """Handle keyboard events."""
        if event.key == ' ':
            self.paused = not self.paused
            print(f"{'Paused' if self.paused else 'Resumed'}")

        elif event.key == 'c':
            self.clear_data()
            print("Data cleared")

        elif event.key == 's':
            self.save_csv()

        elif event.key == 'm':
            self.measure_mode = not self.measure_mode
            self.measure_points = []
            print(f"Measure mode: {'ON - click two points' if self.measure_mode else 'OFF'}")

        elif event.key == 'd':
            # Toggle debug output on device
            if self.ser and self.ser.is_open:
                self.ser.write(b'IRD\r\n')
                print("Sent toggle debug command")

    def on_click(self, event):
        """Handle mouse click for measurement."""
        if not self.measure_mode or event.inaxes is None:
            return

        self.measure_points.append((event.xdata, event.ydata))
        print(f"Point {len(self.measure_points)}: x={event.xdata:.1f}, y={event.ydata:.1f}")

        if len(self.measure_points) >= 2:
            p1, p2 = self.measure_points[-2], self.measure_points[-1]
            dx = abs(p2[0] - p1[0])
            dy = abs(p2[1] - p1[1])
            # Assuming 20Hz (50ms) sampling
            dt_ms = dx * 50
            print(f"Delta: {dx:.1f} samples = {dt_ms:.0f}ms, {dy:.1f}mm")
            self.measure_points = []

    def clear_data(self):
        """Clear all data buffers."""
        self.timestamps.clear()
        self.left_mm.clear()
        self.right_mm.clear()
        self.left_raw.clear()
        self.right_raw.clear()
        self.left_mm_raw.clear()
        self.right_mm_raw.clear()
        self.ir_count = 0
        self.start_time = time.time()
        self.raw_lines = []

    def save_csv(self):
        """Save data to CSV file."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'ir_data_{timestamp}.csv'

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_ms', 'left_mm', 'right_mm', 'left_raw', 'right_raw',
                            'left_mm_raw', 'right_mm_raw'])

            for i in range(len(self.timestamps)):
                writer.writerow([
                    self.timestamps[i] if i < len(self.timestamps) else '',
                    self.left_mm[i] if i < len(self.left_mm) else '',
                    self.right_mm[i] if i < len(self.right_mm) else '',
                    self.left_raw[i] if i < len(self.left_raw) else '',
                    self.right_raw[i] if i < len(self.right_raw) else '',
                    self.left_mm_raw[i] if i < len(self.left_mm_raw) else '',
                    self.right_mm_raw[i] if i < len(self.right_mm_raw) else '',
                ])

        print(f"Saved {len(self.timestamps)} samples to {filename}")

    def run(self):
        """Main loop."""
        if not self.connect():
            return

        # Enable debug output on device
        time.sleep(0.5)
        self.ser.write(b'IRD\r\n')
        print("Sent IR debug enable command")

        ani = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=UPDATE_INTERVAL,
            blit=False,  # blit=False for status text updates
            cache_frame_data=False
        )

        plt.show()

        if self.ser:
            self.ser.close()


def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if not port:
            print("No serial port found. Please specify one:")
            print(f"  {sys.argv[0]} /dev/cu.usbmodemXXXX")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    plotter = IRPlotter(port)
    plotter.run()


if __name__ == '__main__':
    main()
