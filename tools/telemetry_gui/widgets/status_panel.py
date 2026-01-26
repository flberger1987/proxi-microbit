"""
Status Panel Widget showing navigation state and mode.

Displays nav state, mode, and motor status with color coding.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QFont


class StatusPanelWidget(QWidget):
    """Status panel showing navigation state and connection info."""

    # Navigation state colors
    NAV_COLORS = {
        "DISABLED": "#888888",
        "YAW_TEST_CW": "#FF8800",
        "YAW_TEST_CCW": "#FF8800",
        "YAW_TEST_DONE": "#888888",
        "HEADING_HOLD": "#00CC66",
        "TURNING": "#FFAA00",
        "BACKING_UP": "#FF6666",
    }

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setMinimumSize(150, 120)

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Title
        title = QLabel("STATUS")
        title.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        title.setStyleSheet("color: #cccccc;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Nav state
        nav_layout = QHBoxLayout()
        nav_label = QLabel("Nav:")
        nav_label.setFont(QFont("Monospace", 9))
        nav_label.setStyleSheet("color: #aaaaaa;")
        self.nav_value = QLabel("DISABLED")
        self.nav_value.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        self.nav_value.setStyleSheet("color: #888888;")
        nav_layout.addWidget(nav_label)
        nav_layout.addWidget(self.nav_value)
        nav_layout.addStretch()
        layout.addLayout(nav_layout)

        # Mode
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        mode_label.setFont(QFont("Monospace", 9))
        mode_label.setStyleSheet("color: #aaaaaa;")
        self.mode_value = QLabel("MANUAL")
        self.mode_value.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        self.mode_value.setStyleSheet("color: #888888;")
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_value)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        # Motors
        motors_layout = QHBoxLayout()
        motors_label = QLabel("Motors:")
        motors_label.setFont(QFont("Monospace", 9))
        motors_label.setStyleSheet("color: #aaaaaa;")
        self.motors_value = QLabel("OFF")
        self.motors_value.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        self.motors_value.setStyleSheet("color: #888888;")
        motors_layout.addWidget(motors_label)
        motors_layout.addWidget(self.motors_value)
        motors_layout.addStretch()
        layout.addLayout(motors_layout)

        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("background-color: #444444;")
        layout.addWidget(line)

        # Connection
        conn_layout = QHBoxLayout()
        conn_label = QLabel("BLE:")
        conn_label.setFont(QFont("Monospace", 9))
        conn_label.setStyleSheet("color: #aaaaaa;")
        self.conn_value = QLabel("Disconnected")
        self.conn_value.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        self.conn_value.setStyleSheet("color: #FF6666;")
        conn_layout.addWidget(conn_label)
        conn_layout.addWidget(self.conn_value)
        conn_layout.addStretch()
        layout.addLayout(conn_layout)

        # Rate
        rate_layout = QHBoxLayout()
        rate_label = QLabel("Rate:")
        rate_label.setFont(QFont("Monospace", 9))
        rate_label.setStyleSheet("color: #aaaaaa;")
        self.rate_value = QLabel("0.0 Hz")
        self.rate_value.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        self.rate_value.setStyleSheet("color: #888888;")
        rate_layout.addWidget(rate_label)
        rate_layout.addWidget(self.rate_value)
        rate_layout.addStretch()
        layout.addLayout(rate_layout)

        layout.addStretch()

    def update_status(self, nav_state: str, autonav: bool, motors: bool):
        """Update navigation status display."""
        # Nav state
        self.nav_value.setText(nav_state)
        color = self.NAV_COLORS.get(nav_state, "#888888")
        self.nav_value.setStyleSheet(f"color: {color};")

        # Mode
        if autonav:
            self.mode_value.setText("AUTO")
            self.mode_value.setStyleSheet("color: #00CC66;")
        else:
            self.mode_value.setText("MANUAL")
            self.mode_value.setStyleSheet("color: #888888;")

        # Motors
        if motors:
            self.motors_value.setText("ON")
            self.motors_value.setStyleSheet("color: #00CC66;")
        else:
            self.motors_value.setText("OFF")
            self.motors_value.setStyleSheet("color: #888888;")

    def update_connection(self, connected: bool, device_name: str = ""):
        """Update connection status display."""
        if connected:
            self.conn_value.setText(f"Connected")
            self.conn_value.setStyleSheet("color: #00CC66;")
        else:
            self.conn_value.setText("Disconnected")
            self.conn_value.setStyleSheet("color: #FF6666;")

    def update_rate(self, rate_hz: float):
        """Update packet rate display."""
        self.rate_value.setText(f"{rate_hz:.1f} Hz")
        if rate_hz > 15:
            self.rate_value.setStyleSheet("color: #00CC66;")
        elif rate_hz > 5:
            self.rate_value.setStyleSheet("color: #FFAA00;")
        else:
            self.rate_value.setStyleSheet("color: #888888;")
