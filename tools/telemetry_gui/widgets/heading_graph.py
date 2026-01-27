"""
Heading Graph Widget using pyqtgraph.

Shows heading history over time with optional target heading line.
"""

import numpy as np
from typing import Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtGui import QColor
import pyqtgraph as pg


class HeadingGraphWidget(QWidget):
    """Time-series graph of heading values."""

    def __init__(self, history_seconds: float = 10.0, parent=None):
        super().__init__(parent)

        self.history_seconds = history_seconds
        self.max_samples = int(history_seconds * 25)  # ~25 Hz max

        # Data buffers
        self.heading_data = np.zeros(self.max_samples)
        self.target_data = np.full(self.max_samples, np.nan)
        self.time_data = np.linspace(-history_seconds, 0, self.max_samples)
        self.sample_index = 0

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Configure pyqtgraph for dark theme
        pg.setConfigOptions(antialias=True)

        # Create plot widget with dark background
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground(QColor(13, 13, 13))  # #0d0d0d
        self.plot_widget.setTitle("Heading", color='#aaaaaa', size='10pt')

        # Configure axes with gray text
        axis_pen = pg.mkPen(color='#888888')
        self.plot_widget.getAxis('left').setPen(axis_pen)
        self.plot_widget.getAxis('left').setTextPen(pg.mkPen(color='#aaaaaa'))
        self.plot_widget.getAxis('bottom').setPen(axis_pen)
        self.plot_widget.getAxis('bottom').setTextPen(pg.mkPen(color='#aaaaaa'))
        self.plot_widget.setLabel('left', 'Degrees', color='#aaaaaa')
        self.plot_widget.setLabel('bottom', 'Time (s)', color='#aaaaaa')
        self.plot_widget.setYRange(0, 360)
        self.plot_widget.setXRange(-self.history_seconds, 0)

        # Grid
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)

        # Heading line - hacker green
        self.heading_line = self.plot_widget.plot(
            self.time_data,
            self.heading_data,
            pen=pg.mkPen(color=(0, 255, 65), width=2),  # Hacker green
            name='Heading'
        )

        # Target line (dashed)
        self.target_line = self.plot_widget.plot(
            self.time_data,
            self.target_data,
            pen=pg.mkPen(color=(255, 80, 80), width=2, style=pg.QtCore.Qt.PenStyle.DashLine),
            name='Target'
        )

        layout.addWidget(self.plot_widget)

    def add_sample(self, heading: float, target: Optional[float] = None):
        """Add a new heading sample to the graph."""
        # Shift data left
        self.heading_data = np.roll(self.heading_data, -1)
        self.target_data = np.roll(self.target_data, -1)

        # Add new sample at the end
        self.heading_data[-1] = heading
        self.target_data[-1] = target if target is not None else np.nan

        # Update plots
        self.heading_line.setData(self.time_data, self.heading_data)
        self.target_line.setData(self.time_data, self.target_data)

    def clear_data(self):
        """Clear all data from the graph."""
        self.heading_data = np.zeros(self.max_samples)
        self.target_data = np.full(self.max_samples, np.nan)
        self.heading_line.setData(self.time_data, self.heading_data)
        self.target_line.setData(self.time_data, self.target_data)
