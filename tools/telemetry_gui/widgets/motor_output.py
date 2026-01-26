"""
Motor Output Widget showing linear and angular commands.

Displays two horizontal bars with center null line.
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont


class MotorOutputWidget(QWidget):
    """Horizontal bar display for motor commands."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.linear = 0  # -100 to +100
        self.angular = 0  # -100 to +100

        self.setMinimumSize(140, 120)

        # Colors
        self.bg_color = QColor(30, 30, 30)
        self.border_color = QColor(60, 60, 60)
        self.text_color = QColor(200, 200, 200)
        self.positive_color = QColor(0, 200, 80)
        self.negative_color = QColor(255, 80, 80)
        self.center_color = QColor(100, 100, 100)

    def update_motors(self, linear: int, angular: int):
        """Update the displayed motor values."""
        self.linear = max(-100, min(100, linear))
        self.angular = max(-100, min(100, angular))
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
        bar_height = 20
        bar_spacing = 25
        label_width = 35

        # Title
        painter.setPen(self.text_color)
        font = QFont("Monospace", 10, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(
            QRectF(0, margin, width, title_height),
            Qt.AlignmentFlag.AlignCenter,
            "MOTORS"
        )

        # Draw bars
        bar_x = margin + label_width
        bar_width = width - 2 * margin - label_width - 35  # Space for value

        for i, (value, label) in enumerate([(self.linear, "Lin"), (self.angular, "Ang")]):
            y = margin + title_height + 10 + i * bar_spacing

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
            if value != 0:
                fill_width = abs(value) / 100 * (bar_width / 2 - 2)
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

            # Value
            value_color = self.positive_color if value > 0 else (
                self.negative_color if value < 0 else self.text_color
            )
            painter.setPen(value_color)
            font_value = QFont("Monospace", 9, QFont.Weight.Bold)
            painter.setFont(font_value)
            painter.drawText(
                QRectF(bar_x + bar_width + 5, y, 30, bar_height),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                f"{value:+d}"
            )
