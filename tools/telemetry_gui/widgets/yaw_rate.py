"""
Yaw Rate Widget showing rotation speed.

Displays a horizontal gauge with center null line.
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont


class YawRateWidget(QWidget):
    """Horizontal gauge for yaw rate display."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.rate = 0.0  # degrees per second
        self.max_rate = 50.0  # +/- 50 deg/s

        self.setMinimumSize(160, 80)

        # Colors
        self.bg_color = QColor(30, 30, 30)
        self.border_color = QColor(60, 60, 60)
        self.text_color = QColor(200, 200, 200)
        self.positive_color = QColor(0, 200, 80)  # CW (right)
        self.negative_color = QColor(255, 180, 0)  # CCW (left)
        self.center_color = QColor(100, 100, 100)
        self.indicator_color = QColor(0, 180, 255)

    def update_rate(self, rate: float):
        """Update the displayed yaw rate."""
        self.rate = max(-self.max_rate, min(self.max_rate, rate))
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
        gauge_height = 20
        scale_height = 15

        # Title
        painter.setPen(self.text_color)
        font = QFont("Monospace", 10, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(
            QRectF(0, margin, width, title_height),
            Qt.AlignmentFlag.AlignCenter,
            "YAW RATE"
        )

        gauge_y = margin + title_height + 5
        gauge_x = margin + 20
        gauge_width = width - 2 * margin - 40

        # Gauge background
        painter.setPen(QPen(self.border_color, 1))
        painter.setBrush(QBrush(QColor(40, 40, 40)))
        painter.drawRect(QRectF(gauge_x, gauge_y, gauge_width, gauge_height))

        # Center line
        center_x = gauge_x + gauge_width / 2
        painter.setPen(QPen(self.center_color, 2))
        painter.drawLine(int(center_x), int(gauge_y - 3), int(center_x), int(gauge_y + gauge_height + 3))

        # Filled portion
        if abs(self.rate) > 0.5:
            fill_ratio = abs(self.rate) / self.max_rate
            fill_width = fill_ratio * (gauge_width / 2 - 2)

            if self.rate > 0:
                color = self.positive_color
                fill_x = center_x + 1
            else:
                color = self.negative_color
                fill_x = center_x - fill_width - 1

            painter.setBrush(QBrush(color))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(QRectF(
                fill_x,
                gauge_y + 2,
                fill_width,
                gauge_height - 4
            ))

        # Scale labels
        painter.setPen(self.text_color)
        font_small = QFont("Monospace", 8)
        painter.setFont(font_small)

        # Left label (CCW)
        painter.drawText(
            QRectF(gauge_x - 5, gauge_y + gauge_height + 2, 40, scale_height),
            Qt.AlignmentFlag.AlignLeft,
            f"-{int(self.max_rate)}"
        )

        # Center
        painter.drawText(
            QRectF(center_x - 10, gauge_y + gauge_height + 2, 20, scale_height),
            Qt.AlignmentFlag.AlignCenter,
            "0"
        )

        # Right label (CW)
        painter.drawText(
            QRectF(gauge_x + gauge_width - 35, gauge_y + gauge_height + 2, 40, scale_height),
            Qt.AlignmentFlag.AlignRight,
            f"+{int(self.max_rate)}"
        )

        # Direction indicators
        painter.setPen(self.negative_color)
        painter.drawText(
            QRectF(margin, gauge_y, 20, gauge_height),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignCenter,
            "\u25c4"  # Left arrow
        )

        painter.setPen(self.positive_color)
        painter.drawText(
            QRectF(width - margin - 20, gauge_y, 20, gauge_height),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignCenter,
            "\u25ba"  # Right arrow
        )

        # Value display
        value_y = gauge_y + gauge_height + scale_height + 2
        value_color = self.positive_color if self.rate > 0.5 else (
            self.negative_color if self.rate < -0.5 else self.text_color
        )
        painter.setPen(value_color)
        font_value = QFont("Monospace", 11, QFont.Weight.Bold)
        painter.setFont(font_value)
        painter.drawText(
            QRectF(0, value_y, width, 20),
            Qt.AlignmentFlag.AlignCenter,
            f"{self.rate:+.1f} \u00b0/s"
        )
