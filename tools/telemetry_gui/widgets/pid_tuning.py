"""
PID Tuning Panel Widget - Live tuning of heading controller parameters.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QPushButton, QGridLayout
)
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QFont


class PIDTuningWidget(QWidget):
    """Widget for live PID parameter tuning."""

    # Signal emitted when parameters should be sent to robot
    send_parameters = pyqtSignal(dict)  # {'kp': float, 'ki': float, 'kd': float, 'i_max': float, 'd_max': float, 'yaw_max': float}

    # Default values (from autonomous_nav.h)
    DEFAULTS = {
        'kp': 0.5,
        'ki': 0.05,
        'kd': 0.1,
        'i_max': 5.0,
        'd_max': 5.0,
        'yaw_max': 12.0,
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(200)
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(8)

        # Title
        title = QLabel("Heading Controller")
        title.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        title.setStyleSheet("color: #00ff41;")
        layout.addWidget(title)

        # Parameters grid
        params_layout = QGridLayout()
        params_layout.setSpacing(5)

        # Style for spinboxes
        spinbox_style = """
            QDoubleSpinBox {
                background-color: #3c3c3c;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 3px;
                color: #cccccc;
            }
            QDoubleSpinBox:focus {
                border-color: #00ff41;
            }
        """

        # KP
        params_layout.addWidget(QLabel("KP:"), 0, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0.0, 5.0)
        self.kp_spin.setSingleStep(0.1)
        self.kp_spin.setDecimals(2)
        self.kp_spin.setValue(self.DEFAULTS['kp'])
        self.kp_spin.setStyleSheet(spinbox_style)
        self.kp_spin.setToolTip("Proportional gain (0.1-2.0 typical)")
        params_layout.addWidget(self.kp_spin, 0, 1)

        # KI
        params_layout.addWidget(QLabel("KI:"), 1, 0)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0.0, 1.0)
        self.ki_spin.setSingleStep(0.01)
        self.ki_spin.setDecimals(3)
        self.ki_spin.setValue(self.DEFAULTS['ki'])
        self.ki_spin.setStyleSheet(spinbox_style)
        self.ki_spin.setToolTip("Integral gain (0.01-0.1 typical)")
        params_layout.addWidget(self.ki_spin, 1, 1)

        # KD
        params_layout.addWidget(QLabel("KD:"), 2, 0)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0.0, 2.0)
        self.kd_spin.setSingleStep(0.05)
        self.kd_spin.setDecimals(2)
        self.kd_spin.setValue(self.DEFAULTS['kd'])
        self.kd_spin.setStyleSheet(spinbox_style)
        self.kd_spin.setToolTip("Derivative gain (0.05-0.5 typical)")
        params_layout.addWidget(self.kd_spin, 2, 1)

        # I_MAX (integral windup limit)
        params_layout.addWidget(QLabel("I_MAX:"), 3, 0)
        self.i_max_spin = QDoubleSpinBox()
        self.i_max_spin.setRange(0.0, 20.0)
        self.i_max_spin.setSingleStep(0.5)
        self.i_max_spin.setDecimals(1)
        self.i_max_spin.setValue(self.DEFAULTS['i_max'])
        self.i_max_spin.setStyleSheet(spinbox_style)
        self.i_max_spin.setToolTip("Integral windup limit (°/s contribution)")
        params_layout.addWidget(self.i_max_spin, 3, 1)

        # D_MAX (derivative term limit)
        params_layout.addWidget(QLabel("D_MAX:"), 4, 0)
        self.d_max_spin = QDoubleSpinBox()
        self.d_max_spin.setRange(0.0, 20.0)
        self.d_max_spin.setSingleStep(0.5)
        self.d_max_spin.setDecimals(1)
        self.d_max_spin.setValue(self.DEFAULTS['d_max'])
        self.d_max_spin.setStyleSheet(spinbox_style)
        self.d_max_spin.setToolTip("Derivative term limit (°/s contribution)")
        params_layout.addWidget(self.d_max_spin, 4, 1)

        # YAW_MAX (maximum yaw rate)
        params_layout.addWidget(QLabel("YAW_MAX:"), 5, 0)
        self.yaw_max_spin = QDoubleSpinBox()
        self.yaw_max_spin.setRange(1.0, 40.0)
        self.yaw_max_spin.setSingleStep(1.0)
        self.yaw_max_spin.setDecimals(1)
        self.yaw_max_spin.setValue(self.DEFAULTS['yaw_max'])
        self.yaw_max_spin.setStyleSheet(spinbox_style)
        self.yaw_max_spin.setToolTip("Maximum yaw rate command (°/s)")
        params_layout.addWidget(self.yaw_max_spin, 5, 1)

        layout.addLayout(params_layout)

        # Send button
        self.send_btn = QPushButton("▶ Send")
        self.send_btn.setStyleSheet("""
            QPushButton {
                background-color: #1a3d1a;
                border: 1px solid #00ff41;
                border-radius: 3px;
                padding: 8px 15px;
                color: #00ff41;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2a5d2a;
            }
            QPushButton:pressed {
                background-color: #0a2d0a;
            }
            QPushButton:disabled {
                background-color: #2a2a2a;
                border-color: #555555;
                color: #666666;
            }
        """)
        self.send_btn.clicked.connect(self._on_send)
        layout.addWidget(self.send_btn)

        # Reset button
        reset_btn = QPushButton("↺ Reset")
        reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #3c3c3c;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 5px 10px;
                color: #888888;
            }
            QPushButton:hover {
                background-color: #4a4a4a;
                color: #cccccc;
            }
        """)
        reset_btn.clicked.connect(self._on_reset)
        layout.addWidget(reset_btn)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setFont(QFont("Monospace", 8))
        self.status_label.setStyleSheet("color: #888888;")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        layout.addStretch()

    def _on_send(self):
        """Send current parameters to robot."""
        params = self.get_parameters()
        self.send_parameters.emit(params)
        # Status will be set by dashboard after verification

    def _on_reset(self):
        """Reset to default values."""
        self.kp_spin.setValue(self.DEFAULTS['kp'])
        self.ki_spin.setValue(self.DEFAULTS['ki'])
        self.kd_spin.setValue(self.DEFAULTS['kd'])
        self.i_max_spin.setValue(self.DEFAULTS['i_max'])
        self.d_max_spin.setValue(self.DEFAULTS['d_max'])
        self.yaw_max_spin.setValue(self.DEFAULTS['yaw_max'])
        self.status_label.setText("Reset to defaults")
        self.status_label.setStyleSheet("color: #888888;")

    def get_parameters(self) -> dict:
        """Get current parameter values."""
        return {
            'kp': self.kp_spin.value(),
            'ki': self.ki_spin.value(),
            'kd': self.kd_spin.value(),
            'i_max': self.i_max_spin.value(),
            'd_max': self.d_max_spin.value(),
            'yaw_max': self.yaw_max_spin.value(),
        }

    def set_parameters(self, params: dict):
        """Set parameter values (e.g., from robot)."""
        if 'kp' in params:
            self.kp_spin.setValue(params['kp'])
        if 'ki' in params:
            self.ki_spin.setValue(params['ki'])
        if 'kd' in params:
            self.kd_spin.setValue(params['kd'])
        if 'i_max' in params:
            self.i_max_spin.setValue(params['i_max'])
        if 'd_max' in params:
            self.d_max_spin.setValue(params['d_max'])
        if 'yaw_max' in params:
            self.yaw_max_spin.setValue(params['yaw_max'])

    def set_status(self, message: str, is_error: bool = False):
        """Set status message."""
        self.status_label.setText(message)
        if is_error:
            self.status_label.setStyleSheet("color: #ff6666;")
        else:
            self.status_label.setStyleSheet("color: #00ff41;")

    def set_sending(self, sending: bool):
        """Set button to sending state."""
        self.send_btn.setEnabled(not sending)
        if sending:
            self.send_btn.setText("Sending...")
            self.status_label.setText("Sending...")
            self.status_label.setStyleSheet("color: #ffaa00;")
        else:
            self.send_btn.setText("▶ Send")
