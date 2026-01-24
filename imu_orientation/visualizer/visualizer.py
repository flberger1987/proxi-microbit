#!/usr/bin/env python3
"""
IMU Orientation Visualizer

Reads orientation data from micro:bit over serial port and
displays a 3D cube visualization using OpenGL.

Usage:
    python visualizer.py [--port /dev/tty.usbmodem*]

Controls:
    ESC - Quit
    R   - Reset view (not orientation from device)
    C   - Request calibration (prints message, actual trigger is on device)
"""

import argparse
import sys
import pygame
from serial_reader import SerialReader, CalibrationData
from cube_renderer import CubeRenderer


class IMUVisualizer:
    """Main visualizer application combining serial reader and 3D renderer."""

    def __init__(self, port: str = None):
        self.reader = SerialReader(port=port)
        self.renderer = CubeRenderer(width=1024, height=768)
        self.calibrating = False

    def on_calibration(self, cal: CalibrationData):
        """Handle calibration events."""
        self.calibrating = cal.in_progress
        if not cal.in_progress:
            print(f"Calibration complete!")
            print(f"  Offsets: X={cal.offset_x:.3f}, Y={cal.offset_y:.3f}, Z={cal.offset_z:.3f}")

    def run(self):
        """Main application loop."""
        # Set up calibration callback
        self.reader.set_calibration_callback(self.on_calibration)

        # Connect to serial port
        if not self.reader.start():
            print("Failed to connect to micro:bit")
            print("Make sure:")
            print("  1. micro:bit is connected via USB")
            print("  2. Firmware is flashed and running")
            print("  3. No other program is using the serial port")
            return 1

        # Initialize renderer
        if not self.renderer.init():
            print("Failed to initialize renderer")
            self.reader.disconnect()
            return 1

        print("\n=== IMU Orientation Visualizer ===")
        print("Reading orientation data from micro:bit...")
        print("Press Button A on micro:bit to calibrate")
        print("Press ESC to quit\n")

        clock = pygame.time.Clock()

        try:
            while self.renderer.running:
                # Process window events
                if not self.renderer.process_events():
                    break

                # Get latest orientation data
                data = self.reader.get_latest()
                if data:
                    self.renderer.set_orientation(
                        roll=data.roll,
                        pitch=data.pitch,
                        heading=data.heading
                    )

                    # Update window title
                    cal_str = " [CALIBRATING]" if self.calibrating else ""
                    pygame.display.set_caption(
                        f"IMU Visualizer - Roll: {data.roll:6.1f}  "
                        f"Pitch: {data.pitch:6.1f}  "
                        f"Heading: {data.heading:6.1f}{cal_str}"
                    )

                # Render frame
                self.renderer.render()

                # Cap at 60 FPS
                clock.tick(60)

        except KeyboardInterrupt:
            print("\nInterrupted")

        finally:
            self.renderer.cleanup()
            self.reader.disconnect()

        return 0


def main():
    parser = argparse.ArgumentParser(
        description='IMU Orientation Visualizer for micro:bit v2'
    )
    parser.add_argument(
        '--port', '-p',
        help='Serial port (auto-detected if not specified)',
        default=None
    )
    parser.add_argument(
        '--list-ports', '-l',
        action='store_true',
        help='List available serial ports and exit'
    )

    args = parser.parse_args()

    if args.list_ports:
        import serial.tools.list_ports
        print("Available serial ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device}: {port.description}")
        return 0

    visualizer = IMUVisualizer(port=args.port)
    return visualizer.run()


if __name__ == '__main__':
    sys.exit(main())
