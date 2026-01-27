"""
Orientation Widget showing Roll and Pitch.

Displays two horizontal bars with center null line.
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont


class OrientationWidget(QWidget):
    """Horizontal bar display for roll and pitch angles."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.roll = 0.0  # -90 to +90 degrees
        self.pitch = 0.0  # -90 to +90 degrees
        self.max_angle = 90.0

        self.setMinimumSize(200, 100)

        # Colors (hacker green theme)
        self.bg_color = QColor(30, 30, 30)
        self.border_color = QColor(60, 60, 60)
        self.text_color = QColor(200, 200, 200)
        self.positive_color = QColor(0, 255, 65)   # Hacker green
        self.negative_color = QColor(0, 255, 65)   # Hacker green
        self.center_color = QColor(100, 100, 100)

    def update_orientation(self, roll: float, pitch: float):
        """Update the displayed roll and pitch values."""
        self.roll = max(-self.max_angle, min(self.max_angle, roll))
        self.pitch = max(-self.max_angle, min(self.max_angle, pitch))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()

        # Background
        painter.fillRect(self.rect(), self.bg_color)

        # Layout
        margin = 10
        title_height = 20
        bar_height = 18
        bar_spacing = 22
        label_width = 40

        # Title
        painter.setPen(self.text_color)
        font = QFont("Monospace", 10, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(
            QRectF(0, margin, width, title_height),
            Qt.AlignmentFlag.AlignCenter,
            "ROLL / PITCH"
        )

        # Draw bars
        bar_x = margin + label_width
        bar_width = width - 2 * margin - label_width - 55  # Space for value

        for i, (value, label) in enumerate([(self.roll, "Roll"), (self.pitch, "Pitch")]):
            y = margin + title_height + 8 + i * bar_spacing

            # Label
            painter.setPen(self.text_color)
            font_small = QFont("Monospace", 9)
            painter.setFont(font_small)
            painter.drawText(
                QRectF(margin, y, label_width, bar_height),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                label
            )

            # Bar background
            painter.setPen(QPen(self.border_color, 1))
            painter.setBrush(QBrush(QColor(40, 40, 40)))
            painter.drawRect(QRectF(bar_x, y, bar_width, bar_height))

            # Center line
            center_x = bar_x + bar_width / 2
            painter.setPen(QPen(self.center_color, 2))
            painter.drawLine(int(center_x), int(y), int(center_x), int(y + bar_height))

            # Filled portion
            if abs(value) > 0.5:
                fill_ratio = abs(value) / self.max_angle
                fill_width = fill_ratio * (bar_width / 2 - 2)

                if value > 0:
                    color = self.positive_color
                    fill_x = center_x + 1
                else:
                    color = self.negative_color
                    fill_x = center_x - fill_width - 1

                painter.setBrush(QBrush(color))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawRect(QRectF(
                    fill_x,
                    y + 2,
                    fill_width,
                    bar_height - 4
                ))

            # Indicator marker
            indicator_x = center_x + (value / self.max_angle) * (bar_width / 2 - 5)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor(255, 255, 255)))
            painter.drawEllipse(QRectF(indicator_x - 3, y + bar_height / 2 - 3, 6, 6))

            # Value
            value_color = self.positive_color if abs(value) > 1 else self.text_color
            painter.setPen(value_color)
            font_value = QFont("Monospace", 9, QFont.Weight.Bold)
            painter.setFont(font_value)
            painter.drawText(
                QRectF(bar_x + bar_width + 5, y, 50, bar_height),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                f"{value:+.1f}\u00b0"
            )
