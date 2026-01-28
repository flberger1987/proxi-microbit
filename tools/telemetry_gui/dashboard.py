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
    QGridLayout, QGroupBox, QLabel, QStatusBar, QPushButton,
    QTabWidget
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
    ThreadStatsWidget,
    LogAnalysisWidget,
    VarianceDisplayWidget,
    PIDTuningWidget
)
from .ble_bridge import BLEReceiver


class TelemetryLogger:
    """CSV logger for telemetry data."""

    # CSV header matching the console logger format
    CSV_HEADER = (
        "timestamp_ms,roll,pitch,heading,target_heading,yaw_rate,"
        "ir_left_mm,ir_right_mm,motor_linear,motor_angular,"
        "nav_state,autonav_enabled,motors_enabled,"
        "raw_ax,raw_ay,raw_az,raw_mx,raw_my,raw_mz,"
        "pid_kp,pid_ki,pid_kd,pid_imax,pid_dmax,pid_ymax\n"
    )

    def __init__(self):
        self.file = None
        self.filename = None
        self.is_logging = False
        self.packet_count = 0
        self.pid_params = {'kp': 0.5, 'ki': 0.05, 'kd': 0.1, 'i_max': 5.0, 'd_max': 5.0, 'yaw_max': 12.0}

    def set_pid_params(self, params: dict):
        """Update PID parameters for logging."""
        self.pid_params = params.copy()

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
        pid = self.pid_params
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
            f"{raw_mag[0]},{raw_mag[1]},{raw_mag[2]},"
            f"{pid.get('kp', 0):.2f},{pid.get('ki', 0):.3f},{pid.get('kd', 0):.2f},"
            f"{pid.get('i_max', 0):.1f},{pid.get('d_max', 0):.1f},{pid.get('yaw_max', 0):.1f}\n"
        )
        self.file.write(line)
        self.packet_count += 1

        # Flush every 20 packets (~1 second at 20Hz)
        if self.packet_count % 20 == 0:
            self.file.flush()


# Hacker Green accent color
ACCENT_COLOR = "#00ff41"  # Matrix green
ACCENT_DARK = "#00cc33"   # Dimmed green

# Dark theme stylesheet
DARK_STYLESHEET = """
QMainWindow {
    background-color: #0d0d0d;
}
QWidget {
    background-color: #0d0d0d;
    color: #cccccc;
}
QGroupBox {
    border: 1px solid #2a2a2a;
    border-radius: 5px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
    color: #00ff41;
}
QStatusBar {
    background-color: #0a0a0a;
    color: #cccccc;
    border-top: 1px solid #2a2a2a;
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
QTabWidget::pane {
    border: 1px solid #2a2a2a;
    border-radius: 5px;
    background-color: #0d0d0d;
}
QTabWidget::tab-bar {
    left: 0px;
}
QTabBar::tab {
    background-color: #1a1a1a;
    color: #666666;
    border: 1px solid #2a2a2a;
    border-bottom: none;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    padding: 8px 20px;
    margin-right: 2px;
}
QTabBar::tab:selected {
    background-color: #0d0d0d;
    color: #00ff41;
    border-bottom: 2px solid #00ff41;
}
QTabBar::tab:hover:!selected {
    background-color: #2a2a2a;
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

        # Tab container with overlay for controls
        tab_container = QWidget()
        tab_container_layout = QVBoxLayout(tab_container)
        tab_container_layout.setContentsMargins(0, 0, 0, 0)

        # Tab widget (full width)
        self.tab_widget = QTabWidget()
        tab_container_layout.addWidget(self.tab_widget)

        # Overlay for record button - positioned absolutely
        self.controls_overlay = QWidget(tab_container)
        controls_layout = QHBoxLayout(self.controls_overlay)
        controls_layout.setContentsMargins(0, 0, 0, 0)

        # Record button
        self.record_button = QPushButton("● REC")
        self.record_button.setObjectName("recordButton")
        self.record_button.setCheckable(True)
        self.record_button.setFixedWidth(80)
        self.record_button.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        self.record_button.clicked.connect(self._on_record_toggle)
        controls_layout.addWidget(self.record_button)

        self.controls_overlay.adjustSize()
        self.tab_container = tab_container

        main_layout.addWidget(tab_container, 1)

        # Tab 1: Live Telemetry
        live_tab = QWidget()
        self.tab_widget.addTab(live_tab, "📡 Live Telemetry")

        # Tab 2: Log Analysis
        self.log_analysis = LogAnalysisWidget()
        self.tab_widget.addTab(self.log_analysis, "📊 Log Analysis")

        # Content area for live telemetry
        content = live_tab
        content_layout = QHBoxLayout(content)
        content_layout.setContentsMargins(0, 10, 0, 0)
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

        # Far right column (Variance + PID Tuning)
        tuning_col = QVBoxLayout()
        tuning_col.setSpacing(10)

        # Heading Variance Display
        variance_group = QGroupBox("Heading σ")
        variance_layout = QVBoxLayout(variance_group)
        self.variance_display = VarianceDisplayWidget()
        variance_layout.addWidget(self.variance_display)
        tuning_col.addWidget(variance_group)

        # PID Tuning Panel
        pid_group = QGroupBox("PID Tuning")
        pid_layout = QVBoxLayout(pid_group)
        self.pid_tuning = PIDTuningWidget()
        self.pid_tuning.send_parameters.connect(self._on_send_pid)
        self.pid_tuning.refresh_requested.connect(self._on_refresh_pid)
        pid_layout.addWidget(self.pid_tuning)
        tuning_col.addWidget(pid_group)

        # Initialize logger with default PID params
        self.logger.set_pid_params(self.pid_tuning.get_parameters())

        tuning_col.addStretch()
        content_layout.addLayout(tuning_col, 0)  # Minimum width

        # Status bar
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.statusbar.showMessage("Disconnected - Starting scan...")

        # Rate label in status bar (right side)
        self.rate_label = QLabel("0.0 Hz")
        self.rate_label.setFont(QFont("Monospace", 10))
        self.rate_label.setStyleSheet("color: #888888;")
        self.statusbar.addPermanentWidget(self.rate_label)

    def _create_header(self) -> QWidget:
        """Create the header widget."""
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(0, 0, 0, 0)

        # Title
        title = QLabel("Kosmos Proxi Telemetry Dashboard")
        title.setFont(QFont("Monospace", 14, QFont.Weight.Bold))
        title.setStyleSheet("color: #00ff41;")
        layout.addWidget(title)

        # Connection status (after title)
        self.conn_label = QLabel("Disconnected")
        self.conn_label.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        self.conn_label.setStyleSheet("color: #ff6666; margin-left: 20px;")
        layout.addWidget(self.conn_label)

        # Record status label (after connection)
        self.record_label = QLabel("")
        self.record_label.setFont(QFont("Monospace", 9))
        self.record_label.setStyleSheet("color: #888888; margin-left: 15px;")
        layout.addWidget(self.record_label)

        # Stretch to fill remaining space
        layout.addStretch()

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
        self.ble_receiver.text_received.connect(self._on_text_received)
        self.ble_receiver.connection_changed.connect(self._on_connection_changed)
        self.ble_receiver.scan_progress.connect(self._on_scan_progress)
        self.ble_receiver.error_occurred.connect(self._on_error)

        # PID verification state
        self._pending_pid_params = None
        self._awaiting_pid_query = False

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

        # Update variance display
        self.variance_display.add_sample(pkt['heading'], pkt['target_heading'])

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
            self.rate_label.setStyleSheet("color: #00cc66;")
        elif pkt['rate_hz'] > 5:
            self.rate_label.setStyleSheet("color: #ffaa00;")
        else:
            self.rate_label.setStyleSheet("color: #888888;")

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
            # Query current PID parameters from robot
            self._query_pid_on_connect()
        else:
            self.conn_label.setText("Disconnected")
            self.conn_label.setStyleSheet("color: #ff6666;")
            self.statusbar.showMessage("Disconnected")

        self.status_panel.update_connection(connected, device_name)

    def _query_pid_on_connect(self):
        """Query PID parameters from robot after connection."""
        # Small delay to let connection stabilize
        QTimer.singleShot(500, self._send_pidget_command)

    def _send_pidget_command(self):
        """Send PIDGET command to query current PID values."""
        if self.ble_receiver.client and self.ble_receiver.client.is_connected:
            self._awaiting_pid_query = True
            asyncio.ensure_future(self.ble_receiver.send_command("PIDGET"))
            self.pid_tuning.set_status("Querying...", is_error=False)
            # Timeout for query
            QTimer.singleShot(2000, self._pid_query_timeout)

    def _pid_query_timeout(self):
        """Handle PID query timeout."""
        if getattr(self, '_awaiting_pid_query', False):
            self._awaiting_pid_query = False
            self.pid_tuning.set_status("Query timeout - using defaults", is_error=True)

    def _on_refresh_pid(self):
        """Handle manual PID refresh request from user."""
        if not self.ble_receiver.client or not self.ble_receiver.client.is_connected:
            self.pid_tuning.set_status("Not connected!", is_error=True)
            return
        self._send_pidget_command()

    def _on_scan_progress(self, message: str):
        """Handle scan progress update."""
        self.statusbar.showMessage(message)

    def _on_error(self, message: str):
        """Handle error."""
        self.statusbar.showMessage(f"Error: {message}")
        self.conn_label.setText("Error")
        self.conn_label.setStyleSheet("color: #ff6666;")

    def _on_text_received(self, text: str):
        """Handle text response from robot."""
        # Parse PID response: PID:KP=0.50,KI=0.050,KD=0.10,IMAX=5.0,DMAX=5.0,YMAX=12.0
        if text.startswith("PID:"):
            params = {}
            try:
                for part in text[4:].split(","):
                    if "=" in part:
                        key, val = part.split("=", 1)
                        key = key.strip().lower()
                        val = float(val.strip())
                        if key == "kp":
                            params['kp'] = val
                        elif key == "ki":
                            params['ki'] = val
                        elif key == "kd":
                            params['kd'] = val
                        elif key == "imax":
                            params['i_max'] = val
                        elif key == "dmax":
                            params['d_max'] = val
                        elif key == "ymax":
                            params['yaw_max'] = val
            except (ValueError, IndexError):
                pass

            # Check if this is a response to our initial query
            if getattr(self, '_awaiting_pid_query', False) and params:
                self._awaiting_pid_query = False
                # Update widget with values from robot
                self.pid_tuning.set_parameters(params)
                self.logger.set_pid_params(params)
                from datetime import datetime
                timestamp = datetime.now().strftime("%H:%M:%S")
                self.pid_tuning.set_status(f"Loaded from robot @ {timestamp}", is_error=False)
                self.statusbar.showMessage("PID parameters loaded from robot")
                return

            # Verify against pending parameters (after send)
            if self._pending_pid_params and params:
                sent = self._pending_pid_params
                # Check if values match (within tolerance)
                match = (
                    abs(params.get('kp', -1) - sent['kp']) < 0.01 and
                    abs(params.get('ki', -1) - sent['ki']) < 0.001 and
                    abs(params.get('kd', -1) - sent['kd']) < 0.01 and
                    abs(params.get('i_max', -1) - sent['i_max']) < 0.1 and
                    abs(params.get('d_max', -1) - sent['d_max']) < 0.1 and
                    abs(params.get('yaw_max', -1) - sent['yaw_max']) < 0.1
                )

                self._pending_pid_params = None
                self.pid_tuning.set_sending(False)

                if match:
                    from datetime import datetime
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    self.pid_tuning.set_status(f"OK @ {timestamp}", is_error=False)
                    self.statusbar.showMessage("PID parameters applied successfully")
                else:
                    self.pid_tuning.set_status("Mismatch!", is_error=True)
                    self.statusbar.showMessage("Warning: PID values don't match!")

    def _on_send_pid(self, params: dict):
        """Send PID parameters to robot via BLE."""
        # Check if connected
        if not self.ble_receiver.client or not self.ble_receiver.client.is_connected:
            self.statusbar.showMessage("Error: Not connected to robot!")
            self.pid_tuning.set_status("Not connected!", is_error=True)
            return

        # Update logger with new PID params
        self.logger.set_pid_params(params)

        # Set sending state
        self.pid_tuning.set_sending(True)
        self._pending_pid_params = params

        # Format: PID:KP=0.50,KI=0.050,KD=0.10,IMAX=5.0,DMAX=5.0,YMAX=12.0
        cmd = f"PID:KP={params['kp']:.2f},KI={params['ki']:.3f},KD={params['kd']:.2f},IMAX={params['i_max']:.1f},DMAX={params['d_max']:.1f},YMAX={params['yaw_max']:.1f}"
        asyncio.ensure_future(self.ble_receiver.send_command(cmd))
        self.statusbar.showMessage(f"Sending: {cmd}")

        # Start timeout timer (3 seconds)
        QTimer.singleShot(3000, self._pid_send_timeout)

    def _pid_send_timeout(self):
        """Handle PID send timeout."""
        if self._pending_pid_params is not None:
            # No response received
            self._pending_pid_params = None
            self.pid_tuning.set_sending(False)
            self.pid_tuning.set_status("Timeout - no response", is_error=True)
            self.statusbar.showMessage("PID send timeout - no response from robot")

    def _position_controls_overlay(self):
        """Position the controls overlay in top-right of tab container."""
        if hasattr(self, 'controls_overlay') and hasattr(self, 'tab_container'):
            # Just the record button (80px)
            overlay_width = 85
            container_width = self.tab_container.width()
            # Position at top-right, aligned with tab bar
            self.controls_overlay.setFixedWidth(overlay_width)
            self.controls_overlay.move(container_width - overlay_width - 10, 3)
            self.controls_overlay.raise_()

    def resizeEvent(self, event):
        """Handle window resize."""
        super().resizeEvent(event)
        self._position_controls_overlay()

    def showEvent(self, event):
        """Handle window show."""
        super().showEvent(event)
        # Position overlay after window is shown
        QTimer.singleShot(0, self._position_controls_overlay)

    def closeEvent(self, event):
        """Handle window close."""
        # Stop recording if active
        if self.logger.is_logging:
            count = self.logger.stop()
            print(f"Recording stopped on close: {count} packets saved to {self.logger.filename}")

        # Signal the BLE receiver to stop
        asyncio.ensure_future(self.ble_receiver.disconnect())
        event.accept()
