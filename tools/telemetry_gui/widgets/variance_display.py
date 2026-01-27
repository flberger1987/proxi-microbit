"""
Heading Variance Display Widget - Shows rolling 10s heading variance.
"""

from collections import deque
import math

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPainter, QColor, QPen, QBrush


class VarianceDisplayWidget(QWidget):
    """Widget showing real-time heading variance over a rolling window."""

    def __init__(self, window_seconds: float = 10.0, sample_rate: float = 20.0, parent=None):
        super().__init__(parent)
        self.window_size = int(window_seconds * sample_rate)  # ~200 samples
        self.heading_errors = deque(maxlen=self.window_size)
        self.current_variance = 0.0
        self.current_std = 0.0
        self.sample_count = 0

        self.setMinimumSize(120, 80)
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(2)

        # Title
        title = QLabel("Heading σ (10s)")
        title.setFont(QFont("Monospace", 8))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #888888;")
        layout.addWidget(title)

        # Value display
        self.value_label = QLabel("--")
        self.value_label.setFont(QFont("Monospace", 24, QFont.Weight.Bold))
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setStyleSheet("color: #888888;")
        layout.addWidget(self.value_label)

        # Sample count
        self.count_label = QLabel("0 samples")
        self.count_label.setFont(QFont("Monospace", 8))
        self.count_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.count_label.setStyleSheet("color: #555555;")
        layout.addWidget(self.count_label)

    def add_sample(self, heading: float, target_heading: float):
        """Add a new heading sample and update variance."""
        # Skip if no valid target
        if target_heading is None or target_heading < 0:
            return

        # Compute heading error with wraparound
        error = heading - target_heading
        # Wrap to -180..180
        error = (error + 180) % 360 - 180

        self.heading_errors.append(error)
        self.sample_count = len(self.heading_errors)

        # Need at least 10 samples for meaningful variance
        if self.sample_count >= 10:
            # Compute mean and variance
            mean = sum(self.heading_errors) / self.sample_count
            variance = sum((e - mean) ** 2 for e in self.heading_errors) / self.sample_count
            self.current_variance = variance
            self.current_std = math.sqrt(variance)

            self._update_display()

    def _update_display(self):
        """Update the display with current variance."""
        # Format value
        self.value_label.setText(f"{self.current_std:.1f}°")

        # Color based on quality
        if self.current_std < 2.0:
            color = "#00cc66"  # Green - excellent
        elif self.current_std < 5.0:
            color = "#ffaa00"  # Yellow - good
        elif self.current_std < 10.0:
            color = "#ff6666"  # Orange - acceptable
        else:
            color = "#ff0000"  # Red - poor

        self.value_label.setStyleSheet(f"color: {color};")

        # Update sample count
        fill_percent = 100 * self.sample_count / self.window_size
        self.count_label.setText(f"{self.sample_count}/{self.window_size} ({fill_percent:.0f}%)")

    def reset(self):
        """Reset the variance calculation."""
        self.heading_errors.clear()
        self.current_variance = 0.0
        self.current_std = 0.0
        self.sample_count = 0
        self.value_label.setText("--")
        self.value_label.setStyleSheet("color: #888888;")
        self.count_label.setText("0 samples")
