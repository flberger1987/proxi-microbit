"""
IR Sensor Widget showing distance bars.

Displays two vertical bars for left and right IR sensors with color gradient.
"""

from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QLinearGradient, QFont


class IRSensorWidget(QWidget):
    """Dual vertical bar display for IR distance sensors."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.left_mm = 0
        self.right_mm = 0
        self.critical_distance = 150  # mm - full bar, red
        self.warning_distance = 250  # mm - yellow
        self.safe_distance = 500     # mm - green, empty bar

        self.setMinimumSize(120, 180)

        # Colors
        self.bg_color = QColor(30, 30, 30)
        self.border_color = QColor(60, 60, 60)
        self.text_color = QColor(200, 200, 200)

    def update_distances(self, left_mm: int, right_mm: int):
        """Update the displayed distances."""
        self.left_mm = max(0, left_mm)
        self.right_mm = max(0, right_mm)
        self.update()

    def _get_fill_ratio(self, distance_mm: int) -> float:
        """Get bar fill ratio (0-1). 150mm = full bar, >=500mm = empty."""
        if distance_mm >= self.safe_distance:
            return 0.0
        elif distance_mm <= self.critical_distance:
            return 1.0
        else:
            # Linear interpolation: 150mm=1.0, 500mm=0.0
            return 1.0 - (distance_mm - self.critical_distance) / (self.safe_distance - self.critical_distance)

    def _get_bar_color(self, distance_mm: int) -> QColor:
        """Get color based on distance with smooth gradient.

        0 or >=500mm = gray (no reading)
        250-500mm = green (0, 200, 0)
        150-250mm = yellow (255, 200, 0)
        <=150mm = red (255, 0, 0)
        """
        if distance_mm == 0 or distance_mm >= self.safe_distance:
            # Gray - no obstacle detected or no reading
            return QColor(100, 100, 100)
        elif distance_mm >= self.warning_distance:
            # Green to Yellow (250-500mm)
            # Closer to 500mm = more green, closer to 250mm = more yellow
            t = (distance_mm - self.warning_distance) / (self.safe_distance - self.warning_distance)
            return QColor(
                int(255 * (1 - t)),      # 255 -> 0 (yellow to green)
                int(100 + 100 * t),      # 100 -> 200 (brighter green)
                0
            )
        elif distance_mm >= self.critical_distance:
            # Yellow to Red (150-250mm)
            t = (distance_mm - self.critical_distance) / (self.warning_distance - self.critical_distance)
            return QColor(
                255,                      # constant
                int(200 * t),             # 0 -> 200
                0
            )
        else:
            # Red (<=150mm)
            return QColor(255, 0, 0)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()

        # Background
        painter.fillRect(self.rect(), self.bg_color)

        # Layout
        margin = 10
        label_height = 35
        title_height = 20
        bar_width = 35
        bar_spacing = 20
        total_bar_width = 2 * bar_width + bar_spacing

        # Center bars
        bars_x = (width - total_bar_width) / 2
        bar_height = height - 2 * margin - label_height - title_height

        # Title
        painter.setPen(self.text_color)
        font = QFont("Monospace", 10, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(
            QRectF(0, margin, width, title_height),
            Qt.AlignmentFlag.AlignCenter,
            "IR SENSORS"
        )

        bars_y = margin + title_height

        # Draw bars
        for i, (distance, label) in enumerate([(self.left_mm, "L"), (self.right_mm, "R")]):
            x = bars_x + i * (bar_width + bar_spacing)

            # Bar background
            painter.setPen(QPen(self.border_color, 1))
            painter.setBrush(QBrush(QColor(40, 40, 40)))
            painter.drawRect(QRectF(x, bars_y, bar_width, bar_height))

            # Filled portion (closer = more filled, 150mm = full)
            fill_ratio = self._get_fill_ratio(distance)
            fill_height = bar_height * fill_ratio

            if fill_height > 0:
                color = self._get_bar_color(distance)
                painter.setBrush(QBrush(color))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawRect(QRectF(
                    x + 2,
                    bars_y + bar_height - fill_height,
                    bar_width - 4,
                    fill_height
                ))

            # Label (L/R)
            painter.setPen(self.text_color)
            font_small = QFont("Monospace", 9)
            painter.setFont(font_small)
            painter.drawText(
                QRectF(x, bars_y + bar_height + 2, bar_width, 15),
                Qt.AlignmentFlag.AlignCenter,
                label
            )

            # Distance value (same color as bar)
            painter.setPen(self._get_bar_color(distance))
            font_value = QFont("Monospace", 9, QFont.Weight.Bold)
            painter.setFont(font_value)
            painter.drawText(
                QRectF(x, bars_y + bar_height + 16, bar_width, 15),
                Qt.AlignmentFlag.AlignCenter,
                f"{distance}"
            )

        # Units label
        painter.setPen(self.text_color)
        font_small = QFont("Monospace", 8)
        painter.setFont(font_small)
        painter.drawText(
            QRectF(0, height - margin - 5, width, 15),
            Qt.AlignmentFlag.AlignCenter,
            "mm"
        )
