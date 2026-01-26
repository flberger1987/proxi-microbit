"""
Compass Widget for displaying heading.

Shows a circular compass with cardinal directions and heading needle.
"""

import math
from typing import Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QFontMetrics


class CompassWidget(QWidget):
    """Circular compass widget showing heading and optional target."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.target_heading: Optional[float] = None

        self.setMinimumSize(180, 220)

        # Colors (dark theme)
        self.bg_color = QColor(30, 30, 30)
        self.ring_color = QColor(60, 60, 60)
        self.tick_color = QColor(150, 150, 150)
        self.text_color = QColor(200, 200, 200)
        self.needle_color = QColor(0, 180, 255)
        self.target_color = QColor(255, 80, 80)
        self.north_color = QColor(255, 100, 100)

    def update_heading(self, heading: float, target: Optional[float] = None):
        """Update the displayed heading and optional target."""
        self.heading = heading % 360
        self.target_heading = target % 360 if target is not None else None
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Get widget dimensions
        width = self.width()
        height = self.height()

        # Calculate compass circle dimensions
        margin = 10
        text_height = 40
        available_height = height - text_height
        diameter = min(width - 2 * margin, available_height - 2 * margin)
        radius = diameter / 2

        # Center of compass
        cx = width / 2
        cy = margin + radius

        # Draw background
        painter.fillRect(self.rect(), self.bg_color)

        # Draw compass ring
        painter.setPen(QPen(self.ring_color, 3))
        painter.setBrush(QBrush(QColor(40, 40, 40)))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Draw tick marks and labels
        font = QFont("Monospace", 10, QFont.Weight.Bold)
        painter.setFont(font)

        for angle in range(0, 360, 30):
            rad = math.radians(angle - 90)  # -90 to start from top

            # Tick marks
            inner_r = radius - 15
            outer_r = radius - 5

            if angle % 90 == 0:
                # Major ticks (N, E, S, W)
                inner_r = radius - 20
                painter.setPen(QPen(self.tick_color, 2))
            else:
                painter.setPen(QPen(self.tick_color, 1))

            x1 = cx + inner_r * math.cos(rad)
            y1 = cy + inner_r * math.sin(rad)
            x2 = cx + outer_r * math.cos(rad)
            y2 = cy + outer_r * math.sin(rad)
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            # Cardinal direction labels
            if angle % 90 == 0:
                label_r = radius - 30
                lx = cx + label_r * math.cos(rad)
                ly = cy + label_r * math.sin(rad)

                labels = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
                label = labels.get(angle, '')

                if angle == 0:
                    painter.setPen(self.north_color)
                else:
                    painter.setPen(self.text_color)

                fm = QFontMetrics(font)
                text_rect = fm.boundingRect(label)
                painter.drawText(
                    int(lx - text_rect.width() / 2),
                    int(ly + text_rect.height() / 4),
                    label
                )

        # Draw target heading marker (if set)
        if self.target_heading is not None:
            target_rad = math.radians(self.target_heading - 90)
            tx = cx + (radius - 8) * math.cos(target_rad)
            ty = cy + (radius - 8) * math.sin(target_rad)

            painter.setPen(QPen(self.target_color, 2))
            painter.setBrush(QBrush(self.target_color))
            painter.drawEllipse(QPointF(tx, ty), 5, 5)

        # Draw heading needle
        heading_rad = math.radians(self.heading - 90)
        needle_length = radius - 25

        # Needle tip
        nx = cx + needle_length * math.cos(heading_rad)
        ny = cy + needle_length * math.sin(heading_rad)

        # Needle base (triangle)
        base_width = 8
        base_rad1 = heading_rad + math.pi / 2
        base_rad2 = heading_rad - math.pi / 2

        bx1 = cx + base_width * math.cos(base_rad1)
        by1 = cy + base_width * math.sin(base_rad1)
        bx2 = cx + base_width * math.cos(base_rad2)
        by2 = cy + base_width * math.sin(base_rad2)

        # Draw needle
        painter.setPen(QPen(self.needle_color, 2))
        painter.setBrush(QBrush(self.needle_color))
        from PyQt6.QtGui import QPolygonF
        needle = QPolygonF([
            QPointF(nx, ny),
            QPointF(bx1, by1),
            QPointF(bx2, by2)
        ])
        painter.drawPolygon(needle)

        # Center dot
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(QPointF(cx, cy), 6, 6)

        # Draw heading text below compass
        text_y = cy + radius + 15
        painter.setPen(self.text_color)
        font = QFont("Monospace", 12, QFont.Weight.Bold)
        painter.setFont(font)

        hdg_text = f"HDG: {self.heading:.1f}\u00b0"
        fm = QFontMetrics(font)
        painter.drawText(
            int(cx - fm.horizontalAdvance(hdg_text) / 2),
            int(text_y),
            hdg_text
        )

        if self.target_heading is not None:
            text_y += 18
            painter.setPen(self.target_color)
            tgt_text = f"TGT: {self.target_heading:.1f}\u00b0"
            painter.drawText(
                int(cx - fm.horizontalAdvance(tgt_text) / 2),
                int(text_y),
                tgt_text
            )
