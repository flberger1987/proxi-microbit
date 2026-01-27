"""
Log Analysis Widget - Analyze recorded telemetry CSV files.
"""

import os
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton,
    QLabel, QFileDialog, QGroupBox, QTextEdit, QSplitter
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# Hacker Green accent
ACCENT_GREEN = '#00ff41'

# Dark theme for matplotlib
DARK_STYLE = {
    'figure.facecolor': '#0d0d0d',
    'axes.facecolor': '#1a1a1a',
    'axes.edgecolor': '#888888',      # Light gray border
    'axes.labelcolor': '#aaaaaa',     # Light gray labels
    'axes.titlecolor': '#00ff41',     # Hacker green
    'xtick.color': '#aaaaaa',         # Light gray ticks
    'ytick.color': '#aaaaaa',         # Light gray ticks
    'xtick.labelcolor': '#aaaaaa',    # Light gray tick labels
    'ytick.labelcolor': '#aaaaaa',    # Light gray tick labels
    'text.color': '#cccccc',
    'grid.color': '#2a2a2a',
    'grid.alpha': 0.5,
    'legend.facecolor': '#1a1a1a',
    'legend.edgecolor': '#888888',    # Light gray legend border
    'legend.labelcolor': '#cccccc',
}


class LogAnalysisWidget(QWidget):
    """Widget for analyzing recorded telemetry logs."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.df = None
        self.current_file = None
        self._setup_ui()

    def _setup_ui(self):
        """Set up the UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Header with file selection
        header = self._create_header()
        layout.addWidget(header)

        # Splitter for plots and stats
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: Plots
        plot_widget = QWidget()
        plot_widget.setStyleSheet("background-color: #0d0d0d;")
        plot_layout = QVBoxLayout(plot_widget)
        plot_layout.setContentsMargins(0, 0, 0, 0)

        # Create matplotlib figure with dark background
        self.figure = Figure(figsize=(10, 8), dpi=100, facecolor='#0d0d0d')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background-color: #0d0d0d;")
        plot_layout.addWidget(self.canvas)

        splitter.addWidget(plot_widget)

        # Right: Statistics
        stats_widget = QWidget()
        stats_layout = QVBoxLayout(stats_widget)
        stats_layout.setContentsMargins(5, 5, 5, 5)

        stats_group = QGroupBox("Statistics")
        stats_group_layout = QVBoxLayout(stats_group)
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setFont(QFont("Monospace", 9))
        self.stats_text.setStyleSheet("""
            QTextEdit {
                background-color: #252526;
                color: #cccccc;
                border: 1px solid #3c3c3c;
                border-radius: 5px;
            }
        """)
        stats_group_layout.addWidget(self.stats_text)
        stats_layout.addWidget(stats_group)

        splitter.addWidget(stats_widget)
        splitter.setSizes([800, 200])

        layout.addWidget(splitter, 1)

        # Apply dark theme to figure
        self._apply_dark_theme()

    def _create_header(self) -> QWidget:
        """Create the header with file selection."""
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(0, 0, 0, 0)

        # Recent files dropdown
        layout.addWidget(QLabel("Log File:"))
        self.file_combo = QComboBox()
        self.file_combo.setMinimumWidth(400)
        self.file_combo.setStyleSheet("""
            QComboBox {
                background-color: #3c3c3c;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 5px;
                color: #cccccc;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox QAbstractItemView {
                background-color: #3c3c3c;
                color: #cccccc;
                selection-background-color: #094771;
            }
        """)
        self.file_combo.currentIndexChanged.connect(self._on_file_selected)
        layout.addWidget(self.file_combo)

        # Browse button
        browse_btn = QPushButton("Browse...")
        browse_btn.setStyleSheet("""
            QPushButton {
                background-color: #3c3c3c;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 5px 15px;
                color: #cccccc;
            }
            QPushButton:hover {
                background-color: #4a4a4a;
            }
        """)
        browse_btn.clicked.connect(self._on_browse)
        layout.addWidget(browse_btn)

        # Refresh button
        refresh_btn = QPushButton("↻ Refresh")
        refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #3c3c3c;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 5px 15px;
                color: #cccccc;
            }
            QPushButton:hover {
                background-color: #4a4a4a;
            }
        """)
        refresh_btn.clicked.connect(self._refresh_file_list)
        layout.addWidget(refresh_btn)

        layout.addStretch()

        # Export button
        self.export_btn = QPushButton("📷 Export PNG")
        self.export_btn.setEnabled(False)
        self.export_btn.setStyleSheet("""
            QPushButton {
                background-color: #094771;
                border: 1px solid #1177bb;
                border-radius: 3px;
                padding: 5px 15px;
                color: #ffffff;
            }
            QPushButton:hover {
                background-color: #0a5a8a;
            }
            QPushButton:disabled {
                background-color: #3c3c3c;
                border-color: #555555;
                color: #666666;
            }
        """)
        self.export_btn.clicked.connect(self._on_export)
        layout.addWidget(self.export_btn)

        return header

    def _apply_dark_theme(self):
        """Apply dark theme to matplotlib figure."""
        for key, value in DARK_STYLE.items():
            matplotlib.rcParams[key] = value

    def _refresh_file_list(self):
        """Refresh the list of available log files."""
        self.file_combo.clear()
        self.file_combo.addItem("-- Select a log file --", None)

        # Search in current directory and tools directory
        search_paths = [
            Path.cwd(),
            Path(__file__).parent.parent.parent,  # tools/
            Path.home() / "ProxiMicro",
            Path.home() / "ProxiMicro" / "tools",
        ]

        found_files = set()
        for search_path in search_paths:
            if search_path.exists():
                for f in search_path.glob("*_telemetrie.csv"):
                    if f not in found_files:
                        found_files.add(f)

        # Sort by modification time (newest first)
        sorted_files = sorted(found_files, key=lambda x: x.stat().st_mtime, reverse=True)

        for f in sorted_files[:20]:  # Limit to 20 most recent
            # Extract timestamp from filename
            name = f.name
            size_kb = f.stat().st_size / 1024
            self.file_combo.addItem(f"{name} ({size_kb:.1f} KB)", str(f))

    def _on_browse(self):
        """Open file browser to select a log file."""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Telemetry Log",
            str(Path.home() / "ProxiMicro"),
            "CSV Files (*.csv);;All Files (*)"
        )
        if filename:
            self._load_and_analyze(filename)

    def _on_file_selected(self, index: int):
        """Handle file selection from dropdown."""
        if index <= 0:
            return

        filepath = self.file_combo.itemData(index)
        if filepath:
            self._load_and_analyze(filepath)

    def _load_and_analyze(self, filepath: str):
        """Load and analyze a telemetry log file."""
        try:
            self.df = pd.read_csv(filepath)
            self.current_file = Path(filepath)

            # Convert timestamps to relative seconds
            self.df['time_s'] = (self.df['timestamp_ms'] - self.df['timestamp_ms'].iloc[0]) / 1000

            self._update_plots()
            self._update_statistics()
            self.export_btn.setEnabled(True)

        except Exception as e:
            self.stats_text.setText(f"Error loading file:\n{str(e)}")
            self.export_btn.setEnabled(False)

    def _update_plots(self):
        """Update the matplotlib plots."""
        if self.df is None:
            return

        self.figure.clear()
        self.figure.set_facecolor('#0d0d0d')

        # Filter for autonomous mode
        df = self.df
        autonav = df[df['nav_state'].isin(['HEADING_HOLD', 'TURNING', 'SCANNING'])].copy()

        # Use full data if no autonomous data
        if len(autonav) < 10:
            autonav = df.copy()
            title_suffix = " (Full Session)"
        else:
            title_suffix = " (Autonomous)"

        time = autonav['time_s']

        # Create 4 subplots
        axes = self.figure.subplots(4, 1, sharex=True)
        self.figure.subplots_adjust(hspace=0.3, left=0.08, right=0.95, top=0.93, bottom=0.08)

        # Title
        filename = self.current_file.name if self.current_file else "Unknown"
        self.figure.suptitle(f"Telemetry Analysis: {filename}{title_suffix}",
                           fontsize=11, color=ACCENT_GREEN)

        # Helper to style axes
        def style_ax(ax):
            ax.set_facecolor('#1a1a1a')
            for spine in ax.spines.values():
                spine.set_color('#888888')
            ax.tick_params(colors='#aaaaaa')
            ax.xaxis.label.set_color('#aaaaaa')
            ax.yaxis.label.set_color('#aaaaaa')

        # Plot 1: Heading vs Target
        ax1 = axes[0]
        ax1.plot(time, autonav['heading'], '#00ff41', label='Heading', linewidth=1)
        if 'target_heading' in autonav.columns:
            valid_target = autonav[autonav['target_heading'] >= 0]
            if len(valid_target) > 0:
                ax1.plot(valid_target['time_s'], valid_target['target_heading'],
                        '#ff6666', linestyle='--', label='Target', linewidth=1)
        ax1.set_ylabel('Heading (°)')
        ax1.legend(loc='upper right', fontsize=8, facecolor='#1a1a1a', edgecolor='#888888', labelcolor='#cccccc')
        ax1.grid(True, alpha=0.3)
        style_ax(ax1)

        # Plot 2: Heading Error
        ax2 = axes[1]
        if 'target_heading' in autonav.columns:
            valid_target = autonav[autonav['target_heading'] >= 0].copy()
            if len(valid_target) > 0:
                heading_error = valid_target['heading'] - valid_target['target_heading']
                heading_error = (heading_error + 180) % 360 - 180
                ax2.plot(valid_target['time_s'], heading_error, '#00ff41', linewidth=1)
                ax2.axhline(0, color='#ff6666', linestyle='--', linewidth=0.5)
                ax2.axhline(2, color='#ffaa00', linestyle=':', linewidth=0.5, alpha=0.7)
                ax2.axhline(-2, color='#ffaa00', linestyle=':', linewidth=0.5, alpha=0.7)

                # Add sigma annotation
                sigma = heading_error.std()
                ax2.text(0.02, 0.95, f'σ = {sigma:.2f}°',
                        transform=ax2.transAxes, fontsize=9, color='#cccccc',
                        verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='#3c3c3c', edgecolor='#888888'))
        ax2.set_ylabel('Error (°)')
        ax2.set_ylim(-15, 15)
        ax2.grid(True, alpha=0.3)
        style_ax(ax2)

        # Plot 3: Motor Angular + Yaw Rate
        ax3 = axes[2]
        ax3.plot(time, autonav['motor_angular'], '#00cc66', linewidth=1, label='Motor Angular %')
        ax3.plot(time, autonav['yaw_rate'], '#00ff41', linewidth=0.8, alpha=0.7, label='Yaw Rate °/s')
        ax3.axhline(0, color='#ff6666', linestyle='--', linewidth=0.5)
        ax3.set_ylabel('Motor % / Rate')
        ax3.legend(loc='upper right', fontsize=8, facecolor='#1a1a1a', edgecolor='#888888', labelcolor='#cccccc')
        ax3.grid(True, alpha=0.3)
        style_ax(ax3)

        # Plot 4: Roll/Pitch (Tilt)
        ax4 = axes[3]
        ax4.plot(time, autonav['roll'], '#ff6666', linewidth=1, alpha=0.8, label='Roll')
        ax4.plot(time, autonav['pitch'], '#00ff41', linewidth=1, alpha=0.8, label='Pitch')
        ax4.axhline(0, color='#888888', linestyle='--', linewidth=0.5)
        ax4.set_ylabel('Tilt (°)')
        ax4.set_xlabel('Time (s)')
        ax4.legend(loc='upper right', fontsize=8, facecolor='#1a1a1a', edgecolor='#888888', labelcolor='#cccccc')
        ax4.grid(True, alpha=0.3)
        style_ax(ax4)

        self.canvas.draw()

    def _update_statistics(self):
        """Update the statistics text."""
        if self.df is None:
            return

        df = self.df
        autonav = df[df['nav_state'].isin(['HEADING_HOLD', 'TURNING'])]
        heading_hold = df[df['nav_state'] == 'HEADING_HOLD']
        turning = df[df['nav_state'] == 'TURNING']

        stats = []
        stats.append(f"{'='*40}")
        stats.append(f"FILE: {self.current_file.name if self.current_file else 'Unknown'}")
        stats.append(f"{'='*40}")
        stats.append(f"")
        stats.append(f"Duration:     {df['time_s'].max():.1f}s")
        stats.append(f"Samples:      {len(df)}")
        stats.append(f"Sample Rate:  {len(df) / max(df['time_s'].max(), 1):.1f} Hz")
        stats.append(f"")
        stats.append(f"{'─'*40}")
        stats.append(f"AUTONOMOUS MODE")
        stats.append(f"{'─'*40}")
        stats.append(f"Total:        {len(autonav)} ({100*len(autonav)/max(len(df),1):.1f}%)")
        stats.append(f"  HEADING_HOLD: {len(heading_hold)}")
        stats.append(f"  TURNING:      {len(turning)}")

        if len(heading_hold) > 10:
            # Compute heading error
            valid = heading_hold[heading_hold['target_heading'] >= 0]
            if len(valid) > 0:
                heading_error = valid['heading'] - valid['target_heading']
                heading_error = (heading_error + 180) % 360 - 180

                stats.append(f"")
                stats.append(f"{'─'*40}")
                stats.append(f"HEADING HOLD PERFORMANCE")
                stats.append(f"{'─'*40}")
                stats.append(f"Error Mean:   {heading_error.mean():+.2f}°")
                stats.append(f"Error Std:    {heading_error.std():.2f}°")
                stats.append(f"Error Max:    {heading_error.abs().max():.2f}°")

            stats.append(f"")
            stats.append(f"{'─'*40}")
            stats.append(f"TILT DURING WALKING")
            stats.append(f"{'─'*40}")
            stats.append(f"Roll:   {heading_hold['roll'].mean():.1f}° ± {heading_hold['roll'].std():.1f}°")
            stats.append(f"Pitch:  {heading_hold['pitch'].mean():.1f}° ± {heading_hold['pitch'].std():.1f}°")

            stats.append(f"")
            stats.append(f"{'─'*40}")
            stats.append(f"YAW RATE")
            stats.append(f"{'─'*40}")
            stats.append(f"Mean:   {heading_hold['yaw_rate'].mean():.2f}°/s")
            stats.append(f"Std:    {heading_hold['yaw_rate'].std():.2f}°/s")

            stats.append(f"")
            stats.append(f"{'─'*40}")
            stats.append(f"MOTOR ANGULAR OUTPUT")
            stats.append(f"{'─'*40}")
            stats.append(f"Mean:   {heading_hold['motor_angular'].mean():.1f}%")
            stats.append(f"Std:    {heading_hold['motor_angular'].std():.1f}%")

        self.stats_text.setText("\n".join(stats))

    def _on_export(self):
        """Export the current plot to PNG."""
        if self.current_file is None:
            return

        # Default filename based on log file
        default_name = self.current_file.stem + "_analysis.png"
        default_path = self.current_file.parent / default_name

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export Analysis",
            str(default_path),
            "PNG Images (*.png);;All Files (*)"
        )

        if filename:
            self.figure.savefig(filename, dpi=150, facecolor='#1e1e1e', edgecolor='none')

    def showEvent(self, event):
        """Called when the widget is shown."""
        super().showEvent(event)
        # Refresh file list when tab is shown
        self._refresh_file_list()
