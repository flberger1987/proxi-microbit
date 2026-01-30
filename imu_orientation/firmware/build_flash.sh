#!/bin/bash
# build_flash.sh - Build and flash IMU orientation firmware for micro:bit v2
#
# BLE OTA Dual-Slot System (Default):
#   - Custom Mini-Bootloader at 0x00000 (8KB)
#   - Slot A at 0x02000 (244KB) - Primary
#   - Slot B at 0x3F000 (244KB) - Secondary
#   - Boot Control at 0x7C000 (4KB)
#   - OTA State at 0x7D000 (4KB)
#   - Settings at 0x7E000 (8KB)
#
# Usage:
#   ./build_flash.sh              # Build app only (for OTA update)
#   ./build_flash.sh flash        # Initial flash: Bootloader + App via pyOCD
#   ./build_flash.sh ota          # OTA update via BLE (DEFAULT for updates!)
#   ./build_flash.sh ota --backup # OTA with backup of current firmware
#   ./build_flash.sh backup       # Backup current firmware via BLE
#   ./build_flash.sh status       # Query OTA status via BLE
#   ./build_flash.sh bootloader   # Build bootloader only
#   ./build_flash.sh clean        # Clean build directory
#
# Legacy (MCUboot):
#   ./build_flash.sh mcuboot      # Build with MCUboot (sysbuild)
#   ./build_flash.sh flash-mcuboot # Flash MCUboot + signed app

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
BOOTLOADER_SRC="$SCRIPT_DIR/bootloader"
BOOTLOADER_BUILD="$BUILD_DIR/bootloader"
OTA_TOOL="$SCRIPT_DIR/../../tools/ota_upload.py"
ZEPHYR_BASE="${ZEPHYR_BASE:-$HOME/zephyrproject/zephyr}"
ZEPHYR_WORKSPACE="${ZEPHYR_WORKSPACE:-$HOME/zephyrproject}"
BUILD_NUMBER_FILE="$SCRIPT_DIR/.build_number"

# Device settings
DEVICE_NAME="Ozzy"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
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

print_step() {
    echo -e "${CYAN}[STEP]${NC} $1"
}

# Build number management
get_build_number() {
    if [ -f "$BUILD_NUMBER_FILE" ]; then
        cat "$BUILD_NUMBER_FILE"
    else
        echo "0"
    fi
}

increment_build_number() {
    local current=$(get_build_number)
    local next=$((current + 1))
    echo "$next" > "$BUILD_NUMBER_FILE"
    echo "$next"
}

check_dependencies() {
    if [ ! -f "$ZEPHYR_BASE/zephyr-env.sh" ]; then
        print_error "Zephyr not found at $ZEPHYR_BASE"
        print_info "Set ZEPHYR_BASE environment variable or install Zephyr"
        exit 1
    fi
}

check_ota_tool() {
    if [ ! -f "$OTA_TOOL" ]; then
        print_error "OTA upload tool not found at $OTA_TOOL"
        exit 1
    fi

    # Check Python dependencies
    if ! python3 -c "import bleak" 2>/dev/null; then
        print_warn "bleak not installed. Installing..."
        pip3 install bleak tqdm intelhex
    fi
}

# =============================================================================
# Build Functions
# =============================================================================

# Query active slot from device via BLE
# Returns: A or B, or empty if device not found
query_active_slot() {
    python3 -c "
import asyncio
from bleak import BleakClient, BleakScanner

NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'

async def main():
    device = await BleakScanner.find_device_by_name('$DEVICE_NAME', timeout=10)
    if not device:
        return
    response = []
    def handler(s, d):
        t = d.decode('utf-8', errors='ignore')
        if 'SLOT:' in t:
            response.append(t)
    async with BleakClient(device) as client:
        await client.start_notify(NUS_TX, handler)
        await client.write_gatt_char(NUS_RX, b'VER')
        await asyncio.sleep(1)
    if response:
        import re
        m = re.search(r'SLOT:([AB])', response[0])
        if m:
            print(m.group(1))

asyncio.run(main())
" 2>/dev/null
}

# Build for specific slot (A or B)
build_for_slot() {
    local target_slot="$1"
    local build_num=$(increment_build_number)

    print_info "Building for Slot $target_slot..."
    print_info "Build number: $build_num"

    # Activate Zephyr virtual environment if available
    if [ -f "$ZEPHYR_WORKSPACE/.venv/bin/activate" ]; then
        source "$ZEPHYR_WORKSPACE/.venv/bin/activate"
    fi

    # Source Zephyr environment
    source "$ZEPHYR_BASE/zephyr-env.sh"

    # Build with appropriate overlay for target slot
    local extra_overlay=""
    if [ "$target_slot" = "B" ]; then
        extra_overlay="-DEXTRA_DTC_OVERLAY_FILE=$SCRIPT_DIR/boards/slot_b.overlay"
        print_info "Using Slot B overlay (code at 0x3F000)"
    else
        print_info "Using default Slot A (code at 0x02000)"
    fi

    # Clean build to ensure correct linking
    rm -rf "$ZEPHYR_WORKSPACE/build"

    west build -b bbc_microbit_v2 -d "$ZEPHYR_WORKSPACE/build" "$SCRIPT_DIR" --pristine always \
        -- -DBUILD_NUMBER=$build_num $extra_overlay

    # Copy output to local build directory
    mkdir -p "$BUILD_DIR/app"
    cp "$ZEPHYR_WORKSPACE/build/zephyr/zephyr.hex" "$BUILD_DIR/app/" 2>/dev/null || true
    cp "$ZEPHYR_WORKSPACE/build/zephyr/zephyr.bin" "$BUILD_DIR/app/" 2>/dev/null || true

    if [ -f "$BUILD_DIR/app/zephyr.bin" ]; then
        local size=$(stat -f%z "$BUILD_DIR/app/zephyr.bin" 2>/dev/null || stat -c%s "$BUILD_DIR/app/zephyr.bin" 2>/dev/null)
        print_info "Build successful!"
        print_info "  Version: 1.0.0-ble.$build_num"
        print_info "  Target:  Slot $target_slot"
        print_info "  Binary:  $BUILD_DIR/app/zephyr.bin ($size bytes)"
    else
        print_error "Build failed - output not found"
        exit 1
    fi
}

build_app() {
    print_info "Building application for BLE OTA system..."

    # Increment build number
    local build_num=$(increment_build_number)
    print_info "Build number: $build_num"

    # Activate Zephyr virtual environment if available
    if [ -f "$ZEPHYR_WORKSPACE/.venv/bin/activate" ]; then
        source "$ZEPHYR_WORKSPACE/.venv/bin/activate"
    fi

    # Source Zephyr environment
    source "$ZEPHYR_BASE/zephyr-env.sh"

    # Build application (standalone, no sysbuild)
    # Default: Slot A for direct flash
    west build -b bbc_microbit_v2 -d "$ZEPHYR_WORKSPACE/build" "$SCRIPT_DIR" --pristine auto \
        -- -DBUILD_NUMBER=$build_num

    # Copy output to local build directory
    mkdir -p "$BUILD_DIR/app"
    cp "$ZEPHYR_WORKSPACE/build/zephyr/zephyr.hex" "$BUILD_DIR/app/" 2>/dev/null || true
    cp "$ZEPHYR_WORKSPACE/build/zephyr/zephyr.bin" "$BUILD_DIR/app/" 2>/dev/null || true

    if [ -f "$BUILD_DIR/app/zephyr.bin" ]; then
        local size=$(stat -f%z "$BUILD_DIR/app/zephyr.bin" 2>/dev/null || stat -c%s "$BUILD_DIR/app/zephyr.bin" 2>/dev/null)
        print_info "Build successful!"
        print_info "  Version: 1.0.0-ble.$build_num"
        print_info "  Binary:  $BUILD_DIR/app/zephyr.bin ($size bytes)"
        print_info "  HEX:     $BUILD_DIR/app/zephyr.hex"
        print_info ""
        print_info "Next steps:"
        print_info "  Initial flash:  $0 flash"
        print_info "  OTA update:     $0 ota"
    else
        print_error "Build failed - output not found"
        exit 1
    fi
}

build_bootloader() {
    print_info "Building custom bootloader..."

    if [ ! -d "$BOOTLOADER_SRC" ]; then
        print_error "Bootloader source not found at $BOOTLOADER_SRC"
        exit 1
    fi

    # Activate Zephyr virtual environment if available
    if [ -f "$ZEPHYR_WORKSPACE/.venv/bin/activate" ]; then
        source "$ZEPHYR_WORKSPACE/.venv/bin/activate"
    fi

    source "$ZEPHYR_BASE/zephyr-env.sh"

    west build -b bbc_microbit_v2 -d "$BOOTLOADER_BUILD" "$BOOTLOADER_SRC" --pristine auto

    if [ -f "$BOOTLOADER_BUILD/zephyr/zephyr.hex" ]; then
        local size=$(stat -f%z "$BOOTLOADER_BUILD/zephyr/zephyr.bin" 2>/dev/null || stat -c%s "$BOOTLOADER_BUILD/zephyr/zephyr.bin" 2>/dev/null)
        print_info "Bootloader build successful!"
        print_info "  Binary: $BOOTLOADER_BUILD/zephyr/zephyr.hex ($size bytes)"
    else
        print_error "Bootloader build failed"
        exit 1
    fi
}

# =============================================================================
# Flash Functions (pyOCD)
# =============================================================================

flash_initial() {
    print_info "Initial flash: Bootloader + Application via pyOCD"
    print_warn "⚠️  This will ERASE the ENTIRE chip including Settings!"
    print_warn "   Calibration, PID params, and BLE bonds will be LOST!"
    print_info ""
    print_info "Use '$0 flash-app' to update firmware WITHOUT erasing settings."
    echo ""

    if ! command -v pyocd &> /dev/null; then
        print_error "pyOCD not found. Install with: pip install pyocd"
        exit 1
    fi

    # Build bootloader if not present
    if [ ! -f "$BOOTLOADER_BUILD/zephyr/zephyr.hex" ]; then
        print_info "Bootloader not found, building..."
        build_bootloader
    fi

    # Build app if not present
    if [ ! -f "$BUILD_DIR/app/zephyr.hex" ]; then
        print_info "Application not found, building..."
        build_app
    fi

    print_step "1/4: Erasing chip..."
    pyocd erase -t nrf52833 --chip

    print_step "2/4: Flashing bootloader (0x00000)..."
    pyocd flash -t nrf52833 -f 1000000 "$BOOTLOADER_BUILD/zephyr/zephyr.hex"

    print_step "3/4: Flashing application to Slot A (0x02000)..."
    # The app is built to start at 0x02000 via device tree overlay
    pyocd flash -t nrf52833 -f 1000000 "$BUILD_DIR/app/zephyr.hex"

    print_step "4/4: Resetting device..."
    pyocd reset -t nrf52833

    print_info "Initial flash complete!"
    print_info ""
    print_info "Device is now ready for BLE OTA updates."
    print_info "Future updates: $0 ota"
}

flash_app_only() {
    print_info "Flash application only (Settings preserved)"
    print_info "This erases only Slot A and flashes the new firmware."
    print_info "Calibration, PID params, and BLE bonds remain intact."
    echo ""

    if ! command -v pyocd &> /dev/null; then
        print_error "pyOCD not found. Install with: pip install pyocd"
        exit 1
    fi

    # Build app if not present
    if [ ! -f "$BUILD_DIR/app/zephyr.hex" ]; then
        print_info "Application not found, building..."
        build_app
    fi

    # Erase only Slot A (0x02000 - 0x3F000 = 244KB = 0x3D000)
    print_step "1/3: Erasing Slot A (0x02000-0x3F000)..."
    pyocd erase -t nrf52833 --sector 0x02000+0x3D000

    print_step "2/3: Flashing application to Slot A..."
    pyocd flash -t nrf52833 -f 1000000 "$BUILD_DIR/app/zephyr.hex"

    print_step "3/3: Resetting device..."
    pyocd reset -t nrf52833

    print_info "Flash complete! Settings preserved."
}

# =============================================================================
# BLE OTA Functions
# =============================================================================

ota_update() {
    local extra_args="$@"

    print_info "BLE OTA Firmware Update"
    print_info "Device: $DEVICE_NAME"
    echo ""

    check_ota_tool

    # Query active slot from device
    print_info "Querying active slot..."
    local active_slot=$(query_active_slot)

    if [ -z "$active_slot" ]; then
        print_error "Could not query active slot. Is device connected?"
        print_error "Make sure $DEVICE_NAME is powered on and BLE is enabled."
        exit 1
    fi

    # Determine target slot (inactive)
    local target_slot
    if [ "$active_slot" = "A" ]; then
        target_slot="B"
    else
        target_slot="A"
    fi

    print_info "Active slot: $active_slot"
    print_info "Target slot: $target_slot (OTA will write here)"
    echo ""

    # Always rebuild for the correct target slot
    build_for_slot "$target_slot"

    local size=$(stat -f%z "$BUILD_DIR/app/zephyr.bin" 2>/dev/null || stat -c%s "$BUILD_DIR/app/zephyr.bin" 2>/dev/null)
    local build_num=$(get_build_number)
    print_info ""
    print_info "Uploading: $BUILD_DIR/app/zephyr.bin ($size bytes)"
    echo ""

    # Run OTA upload tool
    python3 "$OTA_TOOL" "$BUILD_DIR/app/zephyr.bin" $extra_args
}

ota_backup() {
    print_info "Backing up current firmware via BLE..."

    check_ota_tool

    python3 "$OTA_TOOL" --backup
}

ota_status() {
    print_info "Querying OTA status..."

    check_ota_tool

    python3 "$OTA_TOOL" --status
}

# =============================================================================
# Legacy MCUboot Functions
# =============================================================================

build_mcuboot() {
    print_info "Building firmware with MCUboot (sysbuild)..."
    print_warn "This is legacy mode - consider using BLE OTA system instead"

    if [ -f "$ZEPHYR_WORKSPACE/.venv/bin/activate" ]; then
        source "$ZEPHYR_WORKSPACE/.venv/bin/activate"
    fi

    source "$ZEPHYR_BASE/zephyr-env.sh"

    west build -b bbc_microbit_v2 -d "$BUILD_DIR/mcuboot_build" --sysbuild "$SCRIPT_DIR" --pristine auto

    print_info "MCUboot build complete"
}

flash_mcuboot() {
    print_info "Flashing MCUboot + signed app (legacy mode)..."

    local mcuboot_hex="$BUILD_DIR/mcuboot_build/mcuboot/zephyr/zephyr.hex"
    local app_hex="$BUILD_DIR/mcuboot_build/firmware/zephyr/zephyr.signed.hex"

    if [ ! -f "$mcuboot_hex" ] || [ ! -f "$app_hex" ]; then
        print_error "MCUboot build not found. Run: $0 mcuboot"
        exit 1
    fi

    pyocd erase -t nrf52833 --chip
    pyocd flash -t nrf52833 -f 1000000 "$mcuboot_hex"
    pyocd flash -t nrf52833 -f 1000000 "$app_hex"
    pyocd reset -t nrf52833

    print_info "MCUboot flash complete"
}

# =============================================================================
# Utility Functions
# =============================================================================

clean_build() {
    print_info "Cleaning build directories..."
    rm -rf "$BUILD_DIR"
    rm -rf "$ZEPHYR_WORKSPACE/build"
    print_info "Clean complete"
}

show_usage() {
    echo "╔══════════════════════════════════════════════════════════════════════╗"
    echo "║          BLE OTA Dual-Slot Firmware Build System                     ║"
    echo "╚══════════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Build Commands:"
    echo "  (none)        Build application only (for OTA update)"
    echo "  bootloader    Build custom bootloader only"
    echo "  clean         Remove build directories"
    echo ""
    echo "Flash Commands (pyOCD - requires USB connection):"
    echo "  flash         Initial flash: Bootloader + App (ERASES CHIP!)"
    echo "  flash-app     Flash app only, PRESERVES settings (calibration, PID, bonds)"
    echo ""
    echo "BLE OTA Commands (wireless updates - DEFAULT for updates!):"
    echo "  ota           Upload firmware via BLE OTA"
    echo "  ota --backup  Upload with backup of current firmware first"
    echo "  ota --no-backup  Upload without backup (faster)"
    echo "  backup        Download current firmware from device"
    echo "  status        Query OTA/device status"
    echo ""
    echo "Legacy Commands (MCUboot):"
    echo "  mcuboot       Build with MCUboot (sysbuild)"
    echo "  flash-mcuboot Flash MCUboot + signed app"
    echo ""
    echo "Typical Workflow:"
    echo "  1. First time:     $0 flash     (USB cable required)"
    echo "  2. Build changes:  $0           (build only)"
    echo "  3. Update device:  $0 ota       (wireless!)"
    echo ""
    echo "Flash Layout:"
    echo "  0x00000-0x02000  Bootloader (8KB)"
    echo "  0x02000-0x3F000  Slot A - Primary (244KB)"
    echo "  0x3F000-0x7C000  Slot B - Secondary (244KB)"
    echo "  0x7C000-0x7D000  Boot Control Block (4KB)"
    echo "  0x7D000-0x7E000  OTA State (4KB)"
    echo "  0x7E000-0x80000  Settings/NVS (8KB)"
}

# =============================================================================
# Main
# =============================================================================

check_dependencies

case "${1:-}" in
    # Build commands
    ""|build)
        build_app
        ;;
    bootloader)
        build_bootloader
        ;;
    clean)
        clean_build
        ;;

    # Flash commands (pyOCD)
    flash)
        flash_initial
        ;;
    flash-app)
        flash_app_only
        ;;

    # BLE OTA commands (DEFAULT for updates!)
    ota)
        shift
        ota_update "$@"
        ;;
    backup)
        ota_backup
        ;;
    status)
        ota_status
        ;;

    # Legacy MCUboot
    mcuboot)
        build_mcuboot
        ;;
    flash-mcuboot)
        flash_mcuboot
        ;;

    # Help
    help|--help|-h)
        show_usage
        ;;

    *)
        print_error "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
