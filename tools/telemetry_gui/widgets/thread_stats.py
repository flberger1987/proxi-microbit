"""
Thread Stats Widget showing CPU and stack usage per thread.

Displays a table with thread statistics.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QFont, QBrush


class ThreadStatsWidget(QWidget):
    """Table display for thread statistics."""

    # Thread names (must match firmware)
    THREAD_NAMES = ["main", "motor", "sensor", "ble_ctrl", "telemetry", "audio", "ir_sensors", "autonav", "idle"]

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setMinimumSize(280, 200)

        # Store stats per thread
        self.stats = {}
        for name in self.THREAD_NAMES:
            self.stats[name] = {'cpu': 0.0, 'stack': 0}

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Create table
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setRowCount(len(self.THREAD_NAMES))
        self.table.setHorizontalHeaderLabels(["Thread", "CPU %", "Stack"])

        # Set header style
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)
        self.table.setColumnWidth(1, 60)
        self.table.setColumnWidth(2, 60)

        # Style
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #1e1e1e;
                color: #cccccc;
                gridline-color: #3c3c3c;
                border: none;
            }
            QTableWidget::item {
                padding: 2px;
            }
            QHeaderView::section {
                background-color: #2d2d2d;
                color: #00b4ff;
                padding: 4px;
                border: 1px solid #3c3c3c;
                font-weight: bold;
            }
        """)

        # Initialize rows
        font = QFont("Monospace", 9)
        for row, name in enumerate(self.THREAD_NAMES):
            # Thread name
            item = QTableWidgetItem(name)
            item.setFont(font)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 0, item)

            # CPU %
            item = QTableWidgetItem("--")
            item.setFont(font)
            item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 1, item)

            # Stack
            item = QTableWidgetItem("--")
            item.setFont(font)
            item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 2, item)

        # Disable selection
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # Hide vertical header
        self.table.verticalHeader().setVisible(False)

        layout.addWidget(self.table)

    def update_thread_stats(self, thread_name: str, cpu_percent: float, stack_used: int):
        """Update stats for a single thread."""
        if thread_name not in self.stats:
            return

        self.stats[thread_name] = {'cpu': cpu_percent, 'stack': stack_used}

        # Find row index
        try:
            row = self.THREAD_NAMES.index(thread_name)
        except ValueError:
            return

        # Update CPU cell
        cpu_item = self.table.item(row, 1)
        if cpu_item:
            cpu_item.setText(f"{cpu_percent:.1f}")

            # Color based on CPU usage
            if cpu_percent > 50:
                cpu_item.setForeground(QBrush(QColor(255, 80, 80)))  # Red
            elif cpu_percent > 20:
                cpu_item.setForeground(QBrush(QColor(255, 180, 0)))  # Yellow
            else:
                cpu_item.setForeground(QBrush(QColor(0, 200, 80)))   # Green

        # Update stack cell
        stack_item = self.table.item(row, 2)
        if stack_item:
            stack_item.setText(f"{stack_used}")

            # Color based on stack usage (assuming ~1KB stacks typical)
            if stack_used > 800:
                stack_item.setForeground(QBrush(QColor(255, 80, 80)))  # Red
            elif stack_used > 500:
                stack_item.setForeground(QBrush(QColor(255, 180, 0)))  # Yellow
            else:
                stack_item.setForeground(QBrush(QColor(200, 200, 200)))  # Normal
