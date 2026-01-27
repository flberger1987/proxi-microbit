#!/bin/bash
#
# Start the Kosmos Proxi Telemetry Dashboard
#
# Usage:
#   ./start_dashboard.sh              # Auto-scan for robot
#   ./start_dashboard.sh -a XX:XX:XX  # Connect to specific address
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check for required Python packages
check_deps() {
    python3 -c "import PyQt6, pyqtgraph, qasync, bleak" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "Missing dependencies. Installing..."
        pip install -r "$SCRIPT_DIR/tools/requirements.txt"
    fi
}

check_deps

# Kill any existing dashboard instances
pkill -f "telemetry_receiver.py --gui" 2>/dev/null
pkill -f "telemetry_gui" 2>/dev/null
sleep 0.5

# Run the telemetry receiver in GUI mode
cd "$SCRIPT_DIR/tools"
exec python3 telemetry_receiver.py --gui "$@"
