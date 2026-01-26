#!/bin/bash
# build_flash.sh - Build and flash IMU orientation firmware for micro:bit v2
#
# Usage:
#   ./build_flash.sh              # Build only
#   ./build_flash.sh flash        # Build and flash via pyOCD
#   ./build_flash.sh flash-app    # Flash only application (MCUboot must be present)
#   ./build_flash.sh ble          # Upload via BLE (MCUmgr/SMP)
#   ./build_flash.sh clean        # Clean build directory

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
APP_DIR="$BUILD_DIR/firmware"
MCUBOOT_DIR="$BUILD_DIR/mcuboot"
ZEPHYR_BASE="${ZEPHYR_BASE:-$HOME/zephyrproject/zephyr}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_dependencies() {
    if [ ! -f "$ZEPHYR_BASE/zephyr-env.sh" ]; then
        print_error "Zephyr not found at $ZEPHYR_BASE"
        print_info "Set ZEPHYR_BASE environment variable or install Zephyr"
        exit 1
    fi
}

build_firmware() {
    print_info "Building firmware with MCUboot (sysbuild)..."

    # Source Zephyr environment
    source "$ZEPHYR_BASE/zephyr-env.sh"

    # Build application with sysbuild (includes MCUboot)
    west build -b bbc_microbit_v2 -d "$BUILD_DIR" --sysbuild "$SCRIPT_DIR" --pristine auto

    if [ -f "$APP_DIR/zephyr/zephyr.signed.hex" ]; then
        print_info "Build successful!"
        print_info "  MCUboot:     $MCUBOOT_DIR/zephyr/zephyr.hex"
        print_info "  Application: $APP_DIR/zephyr/zephyr.signed.hex"
        print_info "  OTA Binary:  $APP_DIR/zephyr/zephyr.signed.bin"
    else
        print_error "Build failed - signed image not found"
        print_info "Check build output for errors"
        exit 1
    fi
}

flash_pyocd() {
    print_info "Flashing via pyOCD..."

    # Check if pyOCD is installed
    if ! command -v pyocd &> /dev/null; then
        print_error "pyOCD not found. Install with: pip install pyocd"
        exit 1
    fi

    # Check for hex files
    if [ ! -f "$MCUBOOT_DIR/zephyr/zephyr.hex" ]; then
        print_error "MCUboot not found. Build first with: $0"
        exit 1
    fi

    if [ ! -f "$APP_DIR/zephyr/zephyr.signed.hex" ]; then
        print_error "Application not found. Build first with: $0"
        exit 1
    fi

    print_info "Step 1/4: Erasing chip..."
    pyocd erase -t nrf52833 --chip

    print_info "Step 2/4: Flashing MCUboot bootloader..."
    pyocd flash -t nrf52833 -f 1000000 "$MCUBOOT_DIR/zephyr/zephyr.hex"

    print_info "Step 3/4: Flashing application..."
    pyocd flash -t nrf52833 -f 1000000 "$APP_DIR/zephyr/zephyr.signed.hex"

    print_info "Step 4/4: Resetting device..."
    pyocd reset -t nrf52833

    print_info "Flash complete!"
}

flash_app_only() {
    print_info "Flashing application only (MCUboot must already be present)..."

    if ! command -v pyocd &> /dev/null; then
        print_error "pyOCD not found. Install with: pip install pyocd"
        exit 1
    fi

    if [ ! -f "$APP_DIR/zephyr/zephyr.signed.hex" ]; then
        print_error "Application not found. Build first with: $0"
        exit 1
    fi

    print_info "Flashing application..."
    pyocd flash -t nrf52833 -f 1000000 "$APP_DIR/zephyr/zephyr.signed.hex"

    print_info "Resetting device..."
    pyocd reset -t nrf52833

    print_info "Flash complete!"
}

upload_ble() {
    print_info "Uploading via BLE (MCUmgr/SMP)..."

    # Check for signed binary
    if [ ! -f "$APP_DIR/zephyr/zephyr.signed.bin" ]; then
        print_error "Signed binary not found. Build first with: $0"
        exit 1
    fi

    DEVICE_NAME="Ozzy"

    # Try smpmgr first, then mcumgr
    if command -v smpmgr &> /dev/null; then
        print_info "Using smpmgr..."
        print_info "Uploading firmware to '$DEVICE_NAME'..."
        smpmgr --connstring "ble,name=$DEVICE_NAME" image upload "$APP_DIR/zephyr/zephyr.signed.bin"

        print_info "Resetting device..."
        smpmgr --connstring "ble,name=$DEVICE_NAME" reset

    elif command -v mcumgr &> /dev/null; then
        print_info "Using mcumgr..."
        print_info "Uploading firmware to '$DEVICE_NAME'..."
        mcumgr --conntype ble --connstring "peer_name=$DEVICE_NAME" image upload "$APP_DIR/zephyr/zephyr.signed.bin"

        print_info "Resetting device..."
        mcumgr --conntype ble --connstring "peer_name=$DEVICE_NAME" reset

    else
        print_error "Neither smpmgr nor mcumgr found"
        print_info "Install smpmgr: pip install smpmgr"
        print_info "Or install mcumgr: go install github.com/apache/mynewt-mcumgr-cli/mcumgr@latest"
        exit 1
    fi

    print_info "BLE OTA update complete!"
}

clean_build() {
    print_info "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    print_info "Clean complete"
}

show_usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  (none)      Build firmware with MCUboot (sysbuild)"
    echo "  flash       Build and flash via pyOCD (full flash: MCUboot + app)"
    echo "  flash-app   Flash application only (MCUboot must be present)"
    echo "  ble         Upload new firmware via BLE OTA (MCUmgr/SMP)"
    echo "  clean       Remove build directory"
    echo ""
    echo "Flashing Order (pyOCD):"
    echo "  1. Chip erase"
    echo "  2. Flash MCUboot bootloader"
    echo "  3. Flash signed application"
    echo "  4. Reset device"
    echo ""
    echo "BLE OTA Update:"
    echo "  - Requires device already running with MCUboot + MCUmgr"
    echo "  - Device name: 'Ozzy'"
    echo "  - Uses smpmgr (pip install smpmgr) or mcumgr"
}

# Main
check_dependencies

case "${1:-}" in
    flash)
        build_firmware
        flash_pyocd
        ;;
    flash-app)
        flash_app_only
        ;;
    ble)
        upload_ble
        ;;
    clean)
        clean_build
        ;;
    help|--help|-h)
        show_usage
        ;;
    "")
        build_firmware
        ;;
    *)
        print_error "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac
