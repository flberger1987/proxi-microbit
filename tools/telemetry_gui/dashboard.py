"""
Telemetry Dashboard - Main Window

PyQt6-based dashboard for visualizing Kosmos Proxi robot telemetry.
"""

import asyncio
import os
from datetime import datetime
from typing import Optional

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QStatusBar, QPushButton
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

from .widgets import (
    CompassWidget,
    HeadingGraphWidget,
    IRSensorWidget,
    MotorOutputWidget,
    YawRateWidget,
    OrientationWidget,
    StatusPanelWidget,
    ThreadStatsWidget
)
from .ble_bridge import BLEReceiver


class TelemetryLogger:
    """CSV logger for telemetry data."""

    # CSV header matching the console logger format
    CSV_HEADER = (
        "timestamp_ms,roll,pitch,heading,target_heading,yaw_rate,"
        "ir_left_mm,ir_right_mm,motor_linear,motor_angular,"
        "nav_state,autonav_enabled,motors_enabled,"
        "raw_ax,raw_ay,raw_az,raw_mx,raw_my,raw_mz\n"
    )

    def __init__(self):
        self.file = None
        self.filename = None
        self.is_logging = False
        self.packet_count = 0

    def start(self) -> str:
        """Start logging to a new timestamped CSV file.

        Returns:
            The filename of the created log file.
        """
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"{timestamp}_telemetrie.csv"
        self.file = open(self.filename, 'w')
        self.file.write(self.CSV_HEADER)
        self.file.flush()
        self.is_logging = True
        self.packet_count = 0
        return self.filename

    def stop(self) -> int:
        """Stop logging and close the file.

        Returns:
            The number of packets logged.
        """
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None
        self.is_logging = False
        count = self.packet_count
        return count

    def log_packet(self, pkt: dict):
        """Log a single telemetry packet to CSV."""
        if not self.is_logging or not self.file:
            return

        # Handle target_heading (None -> -1 for CSV)
        target = pkt.get('target_heading')
        target = target if target is not None else -1

        # Get raw sensor data (tuples)
        raw_accel = pkt.get('raw_accel', (0, 0, 0))
        raw_mag = pkt.get('raw_mag', (0, 0, 0))

        # Format the CSV line
        line = (
            f"{pkt.get('timestamp_ms', 0)},"
            f"{pkt.get('roll', 0):.1f},{pkt.get('pitch', 0):.1f},"
            f"{pkt.get('heading', 0):.1f},{target:.1f},"
            f"{pkt.get('yaw_rate', 0):.1f},"
            f"{pkt.get('ir_left_mm', 0)},{pkt.get('ir_right_mm', 0)},"
            f"{pkt.get('motor_linear', 0)},{pkt.get('motor_angular', 0)},"
            f"{pkt.get('nav_state_name', 'UNKNOWN')},"
            f"{int(pkt.get('autonav_enabled', False))},"
            f"{int(pkt.get('motors_enabled', False))},"
            f"{raw_accel[0]},{raw_accel[1]},{raw_accel[2]},"
            f"{raw_mag[0]},{raw_mag[1]},{raw_mag[2]}\n"
        )
        self.file.write(line)
        self.packet_count += 1

        # Flush every 20 packets (~1 second at 20Hz)
        if self.packet_count % 20 == 0:
            self.file.flush()


# Dark theme stylesheet
DARK_STYLESHEET = """
QMainWindow {
    background-color: #1e1e1e;
}
QWidget {
    background-color: #1e1e1e;
    color: #cccccc;
}
QGroupBox {
    border: 1px solid #3c3c3c;
    border-radius: 5px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
    color: #00b4ff;
}
QStatusBar {
    background-color: #252526;
    color: #cccccc;
    border-top: 1px solid #3c3c3c;
}
QLabel {
    color: #cccccc;
}
QPushButton#recordButton {
    background-color: #3c3c3c;
    border: 2px solid #555555;
    border-radius: 5px;
    padding: 5px 15px;
    color: #cccccc;
    font-weight: bold;
}
QPushButton#recordButton:hover {
    background-color: #4a4a4a;
    border-color: #666666;
}
QPushButton#recordButton:checked {
    background-color: #8b0000;
    border-color: #ff4444;
    color: #ffffff;
}
QPushButton#recordButton:checked:hover {
    background-color: #a00000;
}
"""


class TelemetryDashboard(QMainWindow):
    """Main telemetry dashboard window."""

    def __init__(self, ble_receiver: BLEReceiver, parent=None):
        super().__init__(parent)

        self.ble_receiver = ble_receiver
        self.packet_count = 0
        self.last_rate = 0.0

        # CSV Logger
        self.logger = TelemetryLogger()

        self.setWindowTitle("Kosmos Proxi Telemetry Dashboard")
        self.setMinimumSize(1000, 700)
        self.setStyleSheet(DARK_STYLESHEET)

        self._setup_ui()
        self._connect_signals()

    def show(self):
        """Show the dashboard in fullscreen and bring to front."""
        super().showFullScreen()
        self.raise_()
        self.activateWindow()

    def _setup_ui(self):
        """Set up the dashboard UI."""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Header
        header = self._create_header()
        main_layout.addWidget(header)

        # Content area
        content = QWidget()
        content_layout = QHBoxLayout(content)
        content_layout.setSpacing(10)

        # Left column (compass + IR sensors + motors)
        left_col = QVBoxLayout()
        left_col.setSpacing(10)

        # Compass
        compass_group = QGroupBox("Compass")
        compass_layout = QVBoxLayout(compass_group)
        self.compass = CompassWidget()
        compass_layout.addWidget(self.compass)
        left_col.addWidget(compass_group)

        # Bottom row (IR + Motors)
        bottom_left = QHBoxLayout()
        bottom_left.setSpacing(10)

        # IR Sensors
        ir_group = QGroupBox("IR")
        ir_layout = QVBoxLayout(ir_group)
        self.ir_sensors = IRSensorWidget()
        ir_layout.addWidget(self.ir_sensors)
        bottom_left.addWidget(ir_group)

        # Motors
        motor_group = QGroupBox("Motors")
        motor_layout = QVBoxLayout(motor_group)
        self.motors = MotorOutputWidget()
        motor_layout.addWidget(self.motors)
        bottom_left.addWidget(motor_group)

        left_col.addLayout(bottom_left)
        content_layout.addLayout(left_col, 1)

        # Right column (graph + yaw + orientation + status)
        right_col = QVBoxLayout()
        right_col.setSpacing(10)

        # Heading Graph
        graph_group = QGroupBox("Heading History")
        graph_layout = QVBoxLayout(graph_group)
        self.heading_graph = HeadingGraphWidget()
        graph_layout.addWidget(self.heading_graph)
        right_col.addWidget(graph_group, 2)

        # Middle row (Yaw Rate + Orientation)
        middle_right = QHBoxLayout()
        middle_right.setSpacing(10)

        # Yaw Rate
        yaw_group = QGroupBox("Yaw Rate")
        yaw_layout = QVBoxLayout(yaw_group)
        self.yaw_rate = YawRateWidget()
        yaw_layout.addWidget(self.yaw_rate)
        middle_right.addWidget(yaw_group)

        # Orientation
        orient_group = QGroupBox("Orientation")
        orient_layout = QVBoxLayout(orient_group)
        self.orientation = OrientationWidget()
        orient_layout.addWidget(self.orientation)
        middle_right.addWidget(orient_group)

        right_col.addLayout(middle_right, 1)

        # Bottom row (Status + Thread Stats)
        bottom_right = QHBoxLayout()
        bottom_right.setSpacing(10)

        # Status Panel
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        self.status_panel = StatusPanelWidget()
        status_layout.addWidget(self.status_panel)
        bottom_right.addWidget(status_group)

        # Thread Stats
        thread_group = QGroupBox("Threads")
        thread_layout = QVBoxLayout(thread_group)
        self.thread_stats = ThreadStatsWidget()
        thread_layout.addWidget(self.thread_stats)
        bottom_right.addWidget(thread_group)

        right_col.addLayout(bottom_right, 1)

        content_layout.addLayout(right_col, 2)

        main_layout.addWidget(content, 1)

        # Status bar
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.statusbar.showMessage("Disconnected - Starting scan...")

    def _create_header(self) -> QWidget:
        """Create the header widget."""
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(0, 0, 0, 0)

        # Title
        title = QLabel("Kosmos Proxi Telemetry Dashboard")
        title.setFont(QFont("Monospace", 14, QFont.Weight.Bold))
        title.setStyleSheet("color: #00b4ff;")
        layout.addWidget(title)

        layout.addStretch()

        # Record button
        self.record_button = QPushButton("● REC")
        self.record_button.setObjectName("recordButton")
        self.record_button.setCheckable(True)
        self.record_button.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        self.record_button.clicked.connect(self._on_record_toggle)
        layout.addWidget(self.record_button)

        # Record status label
        self.record_label = QLabel("")
        self.record_label.setFont(QFont("Monospace", 9))
        self.record_label.setStyleSheet("color: #888888; margin-left: 10px;")
        layout.addWidget(self.record_label)

        # Spacer
        spacer = QLabel("")
        spacer.setFixedWidth(20)
        layout.addWidget(spacer)

        # Connection status
        self.conn_label = QLabel("Disconnected")
        self.conn_label.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        self.conn_label.setStyleSheet("color: #ff6666;")
        layout.addWidget(self.conn_label)

        # Rate
        self.rate_label = QLabel("0.0 Hz")
        self.rate_label.setFont(QFont("Monospace", 10))
        self.rate_label.setStyleSheet("color: #888888; margin-left: 20px;")
        layout.addWidget(self.rate_label)

        return header

    def _on_record_toggle(self, checked: bool):
        """Handle record button toggle."""
        if checked:
            # Start recording
            filename = self.logger.start()
            self.record_button.setText("■ STOP")
            self.record_label.setText(f"{filename}")
            self.record_label.setStyleSheet("color: #ff4444;")
            self.statusbar.showMessage(f"Recording to {filename}")
        else:
            # Stop recording
            count = self.logger.stop()
            self.record_button.setText("● REC")
            self.record_label.setText(f"Saved {count} pkts")
            self.record_label.setStyleSheet("color: #00cc66;")
            self.statusbar.showMessage(f"Recording stopped: {count} packets saved")

    def _connect_signals(self):
        """Connect BLE receiver signals to widget updates."""
        self.ble_receiver.packet_received.connect(self._on_packet)
        self.ble_receiver.thread_stats_received.connect(self._on_thread_stats)
        self.ble_receiver.connection_changed.connect(self._on_connection_changed)
        self.ble_receiver.scan_progress.connect(self._on_scan_progress)
        self.ble_receiver.error_occurred.connect(self._on_error)

    def _on_packet(self, pkt: dict):
        """Handle incoming telemetry packet."""
        self.packet_count += 1

        # Log to CSV if recording
        if self.logger.is_logging:
            self.logger.log_packet(pkt)
            # Update record label with packet count
            if self.logger.packet_count % 20 == 0:  # Update every ~1 second
                self.record_label.setText(
                    f"{self.logger.filename} ({self.logger.packet_count})"
                )

        # Update compass
        self.compass.update_heading(pkt['heading'], pkt['target_heading'])

        # Update heading graph
        self.heading_graph.add_sample(pkt['heading'], pkt['target_heading'])

        # Update IR sensors
        self.ir_sensors.update_distances(pkt['ir_left_mm'], pkt['ir_right_mm'])

        # Update motors
        self.motors.update_motors(pkt['motor_linear'], pkt['motor_angular'])

        # Update yaw rate
        self.yaw_rate.update_rate(pkt['yaw_rate'])

        # Update orientation
        self.orientation.update_orientation(pkt['roll'], pkt['pitch'])

        # Update status panel
        self.status_panel.update_status(
            pkt['nav_state_name'],
            pkt['autonav_enabled'],
            pkt['motors_enabled']
        )
        self.status_panel.update_rate(pkt['rate_hz'])

        # Update header
        self.last_rate = pkt['rate_hz']
        self.rate_label.setText(f"{pkt['rate_hz']:.1f} Hz")
        if pkt['rate_hz'] > 15:
            self.rate_label.setStyleSheet("color: #00cc66; margin-left: 20px;")
        elif pkt['rate_hz'] > 5:
            self.rate_label.setStyleSheet("color: #ffaa00; margin-left: 20px;")
        else:
            self.rate_label.setStyleSheet("color: #888888; margin-left: 20px;")

    def _on_thread_stats(self, pkt: dict):
        """Handle incoming thread stats packet."""
        self.thread_stats.update_thread_stats(
            pkt['thread_name'],
            pkt['cpu_percent'],
            pkt['stack_used']
        )

    def _on_connection_changed(self, connected: bool, device_name: str):
        """Handle connection state change."""
        if connected:
            self.conn_label.setText(f"Connected: {device_name}")
            self.conn_label.setStyleSheet("color: #00cc66;")
            self.statusbar.showMessage(f"Connected to {device_name}")
        else:
            self.conn_label.setText("Disconnected")
            self.conn_label.setStyleSheet("color: #ff6666;")
            self.statusbar.showMessage("Disconnected")

        self.status_panel.update_connection(connected, device_name)

    def _on_scan_progress(self, message: str):
        """Handle scan progress update."""
        self.statusbar.showMessage(message)

    def _on_error(self, message: str):
        """Handle error."""
        self.statusbar.showMessage(f"Error: {message}")
        self.conn_label.setText("Error")
        self.conn_label.setStyleSheet("color: #ff6666;")

    def closeEvent(self, event):
        """Handle window close."""
        # Stop recording if active
        if self.logger.is_logging:
            count = self.logger.stop()
            print(f"Recording stopped on close: {count} packets saved to {self.logger.filename}")

        # Signal the BLE receiver to stop
        asyncio.ensure_future(self.ble_receiver.disconnect())
        event.accept()
