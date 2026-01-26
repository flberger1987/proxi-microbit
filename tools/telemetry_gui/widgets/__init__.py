"""
Telemetry Dashboard Widgets

Custom PyQt6 widgets for visualizing robot telemetry data.
"""

from .compass import CompassWidget
from .heading_graph import HeadingGraphWidget
from .ir_sensors import IRSensorWidget
from .motor_output import MotorOutputWidget
from .yaw_rate import YawRateWidget
from .orientation import OrientationWidget
from .status_panel import StatusPanelWidget
from .thread_stats import ThreadStatsWidget

__all__ = [
    'CompassWidget',
    'HeadingGraphWidget',
    'IRSensorWidget',
    'MotorOutputWidget',
    'YawRateWidget',
    'OrientationWidget',
    'StatusPanelWidget',
    'ThreadStatsWidget',
]
