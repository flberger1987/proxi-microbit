"""
Telemetry Dashboard - Main Window

PyQt6-based dashboard for visualizing Kosmos Proxi robot telemetry.
"""

import asyncio
from typing import Optional

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QStatusBar
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
"""


class TelemetryDashboard(QMainWindow):
    """Main telemetry dashboard window."""

    def __init__(self, ble_receiver: BLEReceiver, parent=None):
        super().__init__(parent)

        self.ble_receiver = ble_receiver
        self.packet_count = 0
        self.last_rate = 0.0

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
        # Signal the BLE receiver to stop
        asyncio.ensure_future(self.ble_receiver.disconnect())
        event.accept()
