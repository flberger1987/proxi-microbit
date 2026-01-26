"""
Telemetry GUI Module for Kosmos Proxi Robot

Provides a PyQt6-based dashboard for visualizing robot telemetry data.
"""

from .dashboard import TelemetryDashboard
from .ble_bridge import BLEReceiver

__all__ = ['TelemetryDashboard', 'BLEReceiver']
