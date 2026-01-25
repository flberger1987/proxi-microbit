#!/usr/bin/env python3
"""
IMU Data Plotter - Live visualization with Motor PWM and System ID

Parses serial data:
  IMU,<timestamp_ms>,<roll>,<pitch>,<heading>,<yaw_rate>,<motor_pwm>
  MAG,<timestamp_ms>,<raw_mx>,<raw_my>,<raw_mz>
  SYSID,<time_ms>,<pwm>,<yaw_rate>

Controls:
  SPACE     - Pause/Resume
  C         - Clear data
  S         - Save current data to CSV
  M         - Measure mode (click two points to get time delta)

Usage:
    python3 imu_plot.py [port]

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


class IMUPlotter:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.paused = False
        self.measure_mode = False
        self.measure_points = []

        # IMU data buffers
        self.timestamps = deque(maxlen=BUFFER_SIZE)
        self.roll_data = deque(maxlen=BUFFER_SIZE)
        self.pitch_data = deque(maxlen=BUFFER_SIZE)
        self.heading_data = deque(maxlen=BUFFER_SIZE)
        self.yaw_rate_data = deque(maxlen=BUFFER_SIZE)
        self.motor_pwm_data = deque(maxlen=BUFFER_SIZE)

        # Magnetometer raw data buffers
        self.mag_timestamps = deque(maxlen=BUFFER_SIZE)
        self.mag_x = deque(maxlen=BUFFER_SIZE)
        self.mag_y = deque(maxlen=BUFFER_SIZE)
        self.mag_z = deque(maxlen=BUFFER_SIZE)

        # SYSID data buffers
        self.sysid_time = deque(maxlen=BUFFER_SIZE)
        self.sysid_pwm = deque(maxlen=BUFFER_SIZE)
        self.sysid_yaw = deque(maxlen=BUFFER_SIZE)
        self.sysid_active = False

        # Statistics
        self.imu_count = 0
        self.mag_count = 0
        self.start_time = time.time()
        self.raw_lines = []  # Store raw data for CSV export

        # Setup plot with 5 subplots
        self.fig, axes = plt.subplots(5, 1, figsize=(14, 14))
        self.ax1, self.ax2, self.ax3, self.ax4, self.ax5 = axes
        self.fig.suptitle('IMU Orientation + Motor Control  [SPACE=Pause, C=Clear, S=Save, M=Measure]', fontsize=12)

        # Roll/Pitch subplot
        self.ax1.set_ylabel('Degrees')
        self.ax1.set_ylim(-90, 90)
        self.ax1.set_xlim(0, BUFFER_SIZE)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.line_roll, = self.ax1.plot([], [], 'r-', label='Roll', linewidth=1.5)
        self.line_pitch, = self.ax1.plot([], [], 'g-', label='Pitch', linewidth=1.5)
        self.ax1.legend(loc='upper right')
        self.ax1.set_title('Roll / Pitch')

        # Heading subplot
        self.ax2.set_ylabel('Degrees')
        self.ax2.set_ylim(0, 360)
        self.ax2.set_xlim(0, BUFFER_SIZE)
        self.ax2.grid(True, alpha=0.3)
        self.line_heading, = self.ax2.plot([], [], 'b-', label='Heading', linewidth=1.5)
        self.ax2.legend(loc='upper right')
        self.ax2.set_title('Heading (Compass)')

        # Yaw Rate subplot
        self.ax3.set_ylabel('deg/s')
        self.ax3.set_ylim(-50, 50)
        self.ax3.set_xlim(0, BUFFER_SIZE)
        self.ax3.grid(True, alpha=0.3)
        self.ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.ax3.axhline(y=20, color='orange', linestyle=':', alpha=0.7, label='Target Max')
        self.ax3.axhline(y=-20, color='orange', linestyle=':', alpha=0.7)
        self.line_yaw_rate, = self.ax3.plot([], [], 'm-', label='Yaw Rate', linewidth=1.5)
        self.ax3.legend(loc='upper right')
        self.ax3.set_title('Yaw Rate (Target: +/- 20 deg/s)')

        # Motor PWM subplot
        self.ax4.set_ylabel('PWM %')
        self.ax4.set_ylim(-110, 110)
        self.ax4.set_xlim(0, BUFFER_SIZE)
        self.ax4.grid(True, alpha=0.3)
        self.ax4.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.ax4.axhline(y=100, color='red', linestyle=':', alpha=0.5)
        self.ax4.axhline(y=-100, color='red', linestyle=':', alpha=0.5)
        self.line_motor_pwm, = self.ax4.plot([], [], 'c-', label='Motor PWM', linewidth=2)
        self.ax4.legend(loc='upper right')
        self.ax4.set_title('Motor Output (Turn Motor)')

        # Magnetometer subplot
        self.ax5.set_xlabel('Samples')
        self.ax5.set_ylabel('milli-Gauss')
        self.ax5.set_xlim(0, BUFFER_SIZE)
        self.ax5.grid(True, alpha=0.3)
        self.ax5.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.line_mag_x, = self.ax5.plot([], [], 'r-', label='X (forward)', linewidth=1.2)
        self.line_mag_y, = self.ax5.plot([], [], 'g-', label='Y (left)', linewidth=1.2)
        self.line_mag_z, = self.ax5.plot([], [], 'b-', label='Z (up/USB)', linewidth=1.2)
        self.ax5.legend(loc='upper right')
        self.ax5.set_title('Magnetometer (Raw)')

        # Status text
        self.status_text = self.fig.text(0.02, 0.01, '', fontsize=9,
                                          family='monospace')

        # Measurement annotation
        self.measure_annotation = None

        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_key(self, event):
        """Handle keyboard events."""
        if event.key == ' ':
            self.paused = not self.paused
            print(f"{'PAUSED' if self.paused else 'RESUMED'}")
        elif event.key == 'c':
            self.clear_data()
            print("Data cleared")
        elif event.key == 's':
            self.save_to_csv()
        elif event.key == 'm':
            self.measure_mode = not self.measure_mode
            self.measure_points = []
            if self.measure_annotation:
                self.measure_annotation.remove()
                self.measure_annotation = None
            print(f"Measure mode: {'ON - click two points' if self.measure_mode else 'OFF'}")

    def on_click(self, event):
        """Handle mouse click for measurement."""
        if not self.measure_mode or event.inaxes not in [self.ax3, self.ax4]:
            return

        self.measure_points.append((event.xdata, event.ydata))
        print(f"Point {len(self.measure_points)}: x={event.xdata:.1f}")

        if len(self.measure_points) == 2:
            x1, y1 = self.measure_points[0]
            x2, y2 = self.measure_points[1]
            # Calculate time delta (samples at 20Hz = 50ms each)
            samples_delta = abs(x2 - x1)
            time_delta_ms = samples_delta * 50  # 50ms per sample

            msg = f"Delta: {samples_delta:.0f} samples = {time_delta_ms:.0f} ms"
            print(msg)

            # Show annotation on plot
            if self.measure_annotation:
                self.measure_annotation.remove()
            self.measure_annotation = event.inaxes.annotate(
                msg, xy=((x1+x2)/2, (y1+y2)/2),
                fontsize=10, color='red', weight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8)
            )
            self.measure_points = []

    def clear_data(self):
        """Clear all data buffers."""
        for buf in [self.timestamps, self.roll_data, self.pitch_data,
                    self.heading_data, self.yaw_rate_data, self.motor_pwm_data,
                    self.mag_timestamps, self.mag_x, self.mag_y, self.mag_z,
                    self.sysid_time, self.sysid_pwm, self.sysid_yaw]:
            buf.clear()
        self.raw_lines = []
        self.imu_count = 0
        self.mag_count = 0
        self.start_time = time.time()

    def save_to_csv(self):
        """Save data to CSV file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_data_{timestamp}.csv"

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_ms', 'roll', 'pitch', 'heading', 'yaw_rate', 'motor_pwm'])
            for i in range(len(self.timestamps)):
                writer.writerow([
                    list(self.timestamps)[i] if i < len(self.timestamps) else '',
                    list(self.roll_data)[i] if i < len(self.roll_data) else '',
                    list(self.pitch_data)[i] if i < len(self.pitch_data) else '',
                    list(self.heading_data)[i] if i < len(self.heading_data) else '',
                    list(self.yaw_rate_data)[i] if i < len(self.yaw_rate_data) else '',
                    list(self.motor_pwm_data)[i] if i < len(self.motor_pwm_data) else ''
                ])

        print(f"Saved to {filename} ({len(self.timestamps)} samples)")

    def connect(self):
        """Connect to serial port without resetting the device."""
        print(f"Connecting to {self.port}...")

        # WICHTIG: DTR/RTS deaktivieren um Reset zu vermeiden!
        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = BAUD_RATE
        self.ser.timeout = 0.1
        self.ser.dtr = False  # Kein DTR - verhindert Reset!
        self.ser.rts = False  # Kein RTS - verhindert Reset!
        self.ser.open()

        print(f"Connected to {self.port} at {BAUD_RATE} baud")
        print("Waiting for IMU/MAG data...")

    def parse_line(self, line):
        """Parse IMU, MAG, or SYSID data line."""
        try:
            line = line.strip()
            parts = line.split(',')

            if line.startswith('IMU,') and len(parts) >= 5:
                data = {
                    'type': 'IMU',
                    'timestamp': int(parts[1]),
                    'roll': float(parts[2]),
                    'pitch': float(parts[3]),
                    'heading': float(parts[4]),
                    'yaw_rate': 0.0,
                    'motor_pwm': 0.0
                }
                if len(parts) >= 6:
                    data['yaw_rate'] = float(parts[5])
                if len(parts) >= 7:
                    data['motor_pwm'] = float(parts[6])
                return data
            elif line.startswith('MAG,') and len(parts) == 5:
                return {
                    'type': 'MAG',
                    'timestamp': int(parts[1]),
                    'mx': int(parts[2]),
                    'my': int(parts[3]),
                    'mz': int(parts[4])
                }
            elif line.startswith('SYSID,') and len(parts) == 4:
                return {
                    'type': 'SYSID',
                    'time': int(parts[1]),
                    'pwm': int(parts[2]),
                    'yaw_rate': float(parts[3])
                }
        except (ValueError, IndexError):
            pass
        return None

    def read_serial(self):
        """Read and parse serial data."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                data = self.parse_line(line)

                if data and not self.paused:
                    if data['type'] == 'IMU':
                        self.timestamps.append(data['timestamp'])
                        self.roll_data.append(data['roll'])
                        self.pitch_data.append(data['pitch'])
                        self.heading_data.append(data['heading'])
                        self.yaw_rate_data.append(data['yaw_rate'])
                        self.motor_pwm_data.append(data['motor_pwm'])
                        self.imu_count += 1
                        self.raw_lines.append(line.strip())
                    elif data['type'] == 'MAG':
                        self.mag_timestamps.append(data['timestamp'])
                        self.mag_x.append(data['mx'])
                        self.mag_y.append(data['my'])
                        self.mag_z.append(data['mz'])
                        self.mag_count += 1
                    elif data['type'] == 'SYSID':
                        self.sysid_active = True
                        self.sysid_time.append(data['time'])
                        self.sysid_pwm.append(data['pwm'])
                        self.sysid_yaw.append(data['yaw_rate'])
        except serial.SerialException as e:
            print(f"Serial error: {e}")

    def update_plot(self, frame):
        """Animation update function."""
        self.read_serial()

        # Update IMU plots
        if len(self.roll_data) > 0:
            x = range(len(self.roll_data))
            self.line_roll.set_data(x, list(self.roll_data))
            self.line_pitch.set_data(x, list(self.pitch_data))
            self.line_heading.set_data(x, list(self.heading_data))

            latest_roll = self.roll_data[-1]
            latest_pitch = self.pitch_data[-1]
            latest_heading = self.heading_data[-1]

            self.ax1.set_title(f'Roll: {latest_roll:+.1f}° | Pitch: {latest_pitch:+.1f}°')
            self.ax2.set_title(f'Heading: {latest_heading:.1f}°')

        # Update Yaw Rate plot
        if len(self.yaw_rate_data) > 0:
            x = range(len(self.yaw_rate_data))
            self.line_yaw_rate.set_data(x, list(self.yaw_rate_data))
            latest_yaw_rate = self.yaw_rate_data[-1]
            self.ax3.set_title(f'Yaw Rate: {latest_yaw_rate:+.1f} °/s  (Target: ±20)')

            # Auto-scale Y axis for yaw rate
            yaw_min = min(self.yaw_rate_data)
            yaw_max = max(self.yaw_rate_data)
            margin = max(10, (yaw_max - yaw_min) * 0.1)
            self.ax3.set_ylim(min(-30, yaw_min - margin), max(30, yaw_max + margin))

        # Update Motor PWM plot
        if len(self.motor_pwm_data) > 0:
            x = range(len(self.motor_pwm_data))
            self.line_motor_pwm.set_data(x, list(self.motor_pwm_data))
            latest_pwm = self.motor_pwm_data[-1]
            self.ax4.set_title(f'Motor PWM: {latest_pwm:+.0f}%')

        # Update Magnetometer plot
        if len(self.mag_x) > 0:
            x_mag = range(len(self.mag_x))
            self.line_mag_x.set_data(x_mag, list(self.mag_x))
            self.line_mag_y.set_data(x_mag, list(self.mag_y))
            self.line_mag_z.set_data(x_mag, list(self.mag_z))

            # Auto-scale Y axis for magnetometer
            all_mag = list(self.mag_x) + list(self.mag_y) + list(self.mag_z)
            if all_mag:
                mag_min = min(all_mag)
                mag_max = max(all_mag)
                margin = max(50, (mag_max - mag_min) * 0.1)
                self.ax5.set_ylim(mag_min - margin, mag_max + margin)

            latest_mx = self.mag_x[-1]
            latest_my = self.mag_y[-1]
            latest_mz = self.mag_z[-1]

            self.ax5.set_title(f'Raw Magnetometer: X={latest_mx:+d}  Y={latest_my:+d}  Z={latest_mz:+d} mG')

        # Update status
        elapsed = time.time() - self.start_time
        imu_rate = self.imu_count / elapsed if elapsed > 0 else 0
        mag_rate = self.mag_count / elapsed if elapsed > 0 else 0

        pause_status = " [PAUSED]" if self.paused else ""
        measure_status = " [MEASURE]" if self.measure_mode else ""
        status = f"IMU: {self.imu_count} ({imu_rate:.1f} Hz)  |  MAG: {self.mag_count} ({mag_rate:.1f} Hz){pause_status}{measure_status}"
        self.status_text.set_text(status)

        return (self.line_roll, self.line_pitch, self.line_heading, self.line_yaw_rate,
                self.line_motor_pwm, self.line_mag_x, self.line_mag_y, self.line_mag_z,
                self.status_text)

    def run(self):
        """Start the plotter."""
        self.connect()

        # Create animation
        ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )

        plt.tight_layout()
        plt.subplots_adjust(bottom=0.04, top=0.95)

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("\nSerial port closed.")


def main():
    # Get port from argument or auto-detect
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if not port:
            print("Error: No serial port found!")
            print("Usage: python3 imu_plot.py [port]")
            print("Example: python3 imu_plot.py /dev/cu.usbmodem11202")
            sys.exit(1)

    print("=" * 50)
    print("  IMU + Motor Control Plotter")
    print("=" * 50)
    print(f"Port: {port}")
    print()
    print("Controls:")
    print("  SPACE  - Pause/Resume")
    print("  C      - Clear data")
    print("  S      - Save to CSV")
    print("  M      - Measure mode (click 2 points)")
    print()
    print("Press Ctrl+C to exit")
    print()

    plotter = IMUPlotter(port)
    plotter.run()


if __name__ == '__main__':
    main()
