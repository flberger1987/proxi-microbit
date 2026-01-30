#!/usr/bin/env python3
"""
OTA Firmware Upload Tool for Kosmos Proxi Robot

Uploads firmware over BLE using Nordic UART Service (NUS).
Supports Intel HEX and binary firmware files.

Usage:
    python ota_upload.py firmware.hex              # Full update (with backup)
    python ota_upload.py firmware.hex --no-backup  # Skip backup
    python ota_upload.py --backup                  # Only backup current firmware
    python ota_upload.py --status                  # Query OTA status

Requirements:
    pip install bleak tqdm intelhex

Protocol:
    - Commands prefixed with 0xF0
    - 224-byte data blocks
    - CRC16-CCITT for blocks, CRC32 for full image
"""

import argparse
import asyncio
import os
import struct
import sys
import time
from datetime import datetime
from enum import IntEnum
from pathlib import Path
from typing import Optional

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:
    print("Please install bleak: pip install bleak")
    sys.exit(1)

try:
    from tqdm import tqdm
except ImportError:
    # Fallback progress bar if tqdm not available
    class tqdm:
        def __init__(self, total=0, desc="", unit="", unit_scale=False, initial=0, **kwargs):
            self.total = total
            self.desc = desc
            self.n = initial

        def update(self, n=1):
            self.n += n
            pct = (self.n / self.total * 100) if self.total > 0 else 0
            print(f"\r{self.desc}: {pct:.1f}% ({self.n}/{self.total})", end="", flush=True)

        def set_postfix(self, d):
            pass  # Ignore postfix in fallback

        def close(self):
            print()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.close()

try:
    from intelhex import IntelHex
except ImportError:
    IntelHex = None

# BLE UUIDs for Nordic UART Service (NUS)
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write to robot
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Receive from robot

# OTA Protocol Constants
OTA_PREFIX = 0xF0
OTA_DATA_BLOCK_SIZE = 224  # Max data per block (MTU - overhead)
OTA_MAX_RETRIES = 3
# Timeout settings
OTA_TIMEOUT_FAST = 3.0   # seconds - for data blocks (short, with retries)
OTA_TIMEOUT_SLOW = 30.0  # seconds - for init/validate (flash erase takes time)
OTA_VERSION_SIZE = 16

# Device name
DEVICE_NAME = "Ozzy"


class OtaCommand(IntEnum):
    """OTA command codes (sent after 0xF0 prefix)."""
    INIT = 0x01      # Start OTA: size(4B), crc32(4B), version(16B)
    DATA = 0x02      # Send block: block_num(2B), crc16(2B), data(224B)
    QUERY = 0x03     # Query status
    ABORT = 0x04     # Abort transfer
    VALIDATE = 0x05  # Trigger CRC validation
    BACKUP_REQ = 0x06   # Request backup
    BACKUP_ACK = 0x07   # Acknowledge backup block


class OtaResponse(IntEnum):
    """OTA response codes."""
    READY = 0x01       # Ready status struct
    ACK = 0x02         # Block acknowledged
    NAK = 0x03         # Block rejected (retransmit)
    VALID_OK = 0x05    # Validation successful
    VALID_FAIL = 0x06  # Validation failed
    BACKUP_DATA = 0x07 # Backup data block
    BACKUP_DONE = 0x08 # Backup complete


class OtaError(IntEnum):
    """OTA error codes (in NAK response)."""
    NONE = 0x00
    CRC_MISMATCH = 0x01
    SEQUENCE_ERROR = 0x02
    FLASH_ERROR = 0x03
    SIZE_EXCEEDED = 0x04
    NOT_INITIALIZED = 0x05
    TIMEOUT = 0x06
    UNKNOWN = 0xFF


# CRC16-CCITT lookup table (polynomial 0x1021)
CRC16_TABLE = None


def _init_crc16_table():
    """Initialize CRC16-CCITT lookup table."""
    global CRC16_TABLE
    if CRC16_TABLE is not None:
        return

    CRC16_TABLE = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        CRC16_TABLE.append(crc)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """Calculate CRC16-CCITT checksum."""
    _init_crc16_table()
    crc = initial
    for byte in data:
        crc = ((crc << 8) ^ CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


def crc32(data: bytes) -> int:
    """Calculate CRC32 checksum (standard polynomial)."""
    import binascii
    return binascii.crc32(data) & 0xFFFFFFFF


def load_firmware(filepath: str) -> tuple[bytes, str]:
    """
    Load firmware from file.

    Args:
        filepath: Path to .hex or .bin file

    Returns:
        Tuple of (firmware_bytes, detected_version)
    """
    path = Path(filepath)

    if not path.exists():
        raise FileNotFoundError(f"Firmware file not found: {filepath}")

    if path.suffix.lower() == '.hex':
        if IntelHex is None:
            raise ImportError(
                "Intel HEX support requires intelhex package.\n"
                "Install with: pip install intelhex"
            )
        ih = IntelHex(str(path))
        # Get binary data from HEX file
        start_addr = ih.minaddr()
        end_addr = ih.maxaddr()
        firmware = bytes(ih.tobinarray(start=start_addr, end=end_addr))
    elif path.suffix.lower() == '.bin':
        firmware = path.read_bytes()
    else:
        # Try to detect format
        content = path.read_bytes()
        if content.startswith(b':'):
            # Looks like Intel HEX
            if IntelHex is None:
                raise ImportError("Intel HEX support requires: pip install intelhex")
            ih = IntelHex(str(path))
            start_addr = ih.minaddr()
            end_addr = ih.maxaddr()
            firmware = bytes(ih.tobinarray(start=start_addr, end=end_addr))
        else:
            # Assume binary
            firmware = content

    # Try to detect version string in firmware
    # Look for unique "FWVER=" marker (won't conflict with format strings)
    version = "unknown"
    marker = b"FWVER="
    idx = firmware.find(marker)
    if idx >= 0:
        # Extract version string (null-terminated or until non-printable)
        ver_start = idx + len(marker)
        ver_bytes = []
        for i in range(ver_start, min(ver_start + 32, len(firmware))):
            c = firmware[i]
            if c == 0 or c < 32 or c > 126:
                break
            ver_bytes.append(c)
        if ver_bytes:
            version = bytes(ver_bytes).decode('ascii', errors='ignore').strip()

    return firmware, version


class OtaUploader:
    """OTA firmware uploader over BLE NUS."""

    def __init__(self, address: Optional[str] = None):
        self.address = address
        self.client: Optional[BleakClient] = None
        self.response_event = asyncio.Event()
        self.response_data: Optional[bytes] = None
        self.last_block_acked = -1
        self.connected = False

    async def find_device(self) -> Optional[str]:
        """Scan for the robot device."""
        print(f"Scanning for {DEVICE_NAME}...")
        devices = await BleakScanner.discover(timeout=10.0)

        for device in devices:
            if device.name and DEVICE_NAME in device.name:
                print(f"Found: {device.name} ({device.address})")
                return device.address

        # Show all discovered devices
        print(f"\nFound {len(devices)} BLE devices:")
        for device in devices:
            name = device.name or "(no name)"
            print(f"  - {name}: {device.address}")

        return None

    def _notification_handler(self, sender, data: bytes):
        """Handle incoming BLE notifications."""
        # Check for OTA response (starts with 0xF0)
        if len(data) >= 2 and data[0] == OTA_PREFIX:
            self.response_data = data
            self.response_event.set()

    async def connect(self) -> bool:
        """Connect to the device."""
        if self.address is None:
            self.address = await self.find_device()
            if self.address is None:
                print(f"Device '{DEVICE_NAME}' not found!")
                return False

        print(f"Connecting to {self.address}...")

        try:
            self.client = BleakClient(self.address)
            await self.client.connect(timeout=15.0)

            if not self.client.is_connected:
                print("Failed to connect!")
                return False

            # Start notifications
            await self.client.start_notify(NUS_TX_UUID, self._notification_handler)
            self.connected = True
            print("Connected!")
            return True

        except BleakError as e:
            print(f"Connection error: {e}")
            return False

    async def disconnect(self):
        """Disconnect from device."""
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(NUS_TX_UUID)
            except Exception:
                pass
            await self.client.disconnect()
        self.connected = False

    async def _send_command(self, data: bytes, timeout: float = OTA_TIMEOUT_FAST) -> Optional[bytes]:
        """Send command and wait for response."""
        if not self.client or not self.client.is_connected:
            return None

        self.response_event.clear()
        self.response_data = None

        try:
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=False)
        except BleakError as e:
            print(f"Write error: {e}")
            return None

        try:
            await asyncio.wait_for(self.response_event.wait(), timeout=timeout)
            return self.response_data
        except asyncio.TimeoutError:
            return None

    async def query_status(self) -> Optional[dict]:
        """Query OTA status from device."""
        cmd = bytes([OTA_PREFIX, OtaCommand.QUERY])
        response = await self._send_command(cmd)

        if response is None:
            print("No response (timeout)")
            return None

        if len(response) < 2:
            print(f"Invalid response: {response.hex()}")
            return None

        resp_code = response[1]

        if resp_code == OtaResponse.READY:
            # Parse status struct
            # Expected: prefix(1), code(1), state(1), last_block(2), total_blocks(2),
            #           expected_size(4), received_size(4), crc32(4)
            if len(response) >= 18:
                fields = struct.unpack('<BBBHHIIIs', response[:18] + b'\x00')
                return {
                    'state': fields[2],
                    'last_block': fields[3],
                    'total_blocks': fields[4],
                    'expected_size': fields[5],
                    'received_size': fields[6],
                    'expected_crc': fields[7],
                }
            return {'raw': response.hex()}
        else:
            return {'response_code': resp_code, 'raw': response.hex()}

    async def init_transfer(
        self,
        size: int,
        crc: int,
        version: str
    ) -> bool:
        """Initialize OTA transfer."""
        # Pad/truncate version to 16 bytes
        ver_bytes = version.encode('utf-8')[:OTA_VERSION_SIZE]
        ver_bytes = ver_bytes.ljust(OTA_VERSION_SIZE, b'\x00')

        # Build INIT command: prefix(1), cmd(1), size(4), crc32(4), version(16)
        cmd = struct.pack(
            '<BBII',
            OTA_PREFIX,
            OtaCommand.INIT,
            size,
            crc
        ) + ver_bytes

        print(f"Initializing transfer: {size} bytes, CRC32=0x{crc:08X}")
        # Use long timeout - init erases flash which takes time
        response = await self._send_command(cmd, timeout=OTA_TIMEOUT_SLOW)

        if response is None:
            print("Init timeout - no response from device")
            return False

        if len(response) >= 2 and response[1] == OtaResponse.READY:
            print("Device ready for transfer")
            return True
        elif len(response) >= 2 and response[1] == OtaResponse.NAK:
            error = response[4] if len(response) > 4 else 0xFF
            print(f"Init rejected: error={OtaError(error).name}")
            return False
        else:
            print(f"Unexpected response: {response.hex()}")
            return False

    async def send_block(
        self,
        block_num: int,
        data: bytes,
        retries: int = OTA_MAX_RETRIES
    ) -> bool:
        """Send a data block with retries."""
        # Pad data to full block size if needed (use 0xFF like erased flash)
        if len(data) < OTA_DATA_BLOCK_SIZE:
            data = data.ljust(OTA_DATA_BLOCK_SIZE, b'\xFF')

        # Calculate CRC16 for block AFTER padding
        # The firmware calculates CRC on the full 224-byte block
        block_crc = crc16_ccitt(data)

        # Build DATA command: prefix(1), cmd(1), block_num(2), crc16(2), data(224)
        cmd = struct.pack(
            '<BBHH',
            OTA_PREFIX,
            OtaCommand.DATA,
            block_num,
            block_crc
        ) + data

        # Exponential backoff: 3s, 9s, 27s
        base_timeout = OTA_TIMEOUT_FAST

        for attempt in range(retries):
            # Calculate timeout with exponential backoff
            timeout = base_timeout * (3 ** attempt)  # 3s, 9s, 27s
            response = await self._send_command(cmd, timeout=timeout)

            if response is None:
                if attempt < retries - 1:
                    print(f"\rBlock {block_num}: timeout ({timeout:.0f}s), retry {attempt + 1}")
                    await asyncio.sleep(0.5)  # Short pause before retry
                    continue
                return False

            if len(response) >= 4:
                resp_code = response[1]
                resp_block = struct.unpack('<H', response[2:4])[0]

                if resp_code == OtaResponse.ACK and resp_block == block_num:
                    self.last_block_acked = block_num
                    # Delay to let firmware process flash operations
                    # Flash write can block BLE, so give it breathing room
                    await asyncio.sleep(0.03)  # 30ms between blocks
                    return True
                elif resp_code == OtaResponse.NAK:
                    error = response[4] if len(response) > 4 else 0xFF
                    if attempt < retries - 1:
                        print(f"\rBlock {block_num}: NAK error={error}, retry {attempt + 1}")
                        await asyncio.sleep(0.2)
                        continue
                    print(f"\rBlock {block_num}: NAK error={OtaError(error).name}")
                    return False

            if attempt < retries - 1:
                await asyncio.sleep(0.2)

        return False

    async def validate(self) -> tuple[bool, Optional[tuple[int, int]]]:
        """Trigger CRC validation on device."""
        cmd = bytes([OTA_PREFIX, OtaCommand.VALIDATE])
        # Use long timeout - CRC validation reads entire firmware
        response = await self._send_command(cmd, timeout=OTA_TIMEOUT_SLOW)

        if response is None:
            return False, None

        if len(response) >= 2:
            resp_code = response[1]

            if resp_code == OtaResponse.VALID_OK:
                print("Validation successful!")
                return True, None
            elif resp_code == OtaResponse.VALID_FAIL:
                if len(response) >= 10:
                    expected, actual = struct.unpack('<II', response[2:10])
                    print(f"Validation FAILED!")
                    print(f"  Expected CRC: 0x{expected:08X}")
                    print(f"  Actual CRC:   0x{actual:08X}")
                    return False, (expected, actual)
                return False, None

        return False, None

    async def abort(self):
        """Abort current transfer."""
        cmd = bytes([OTA_PREFIX, OtaCommand.ABORT])
        await self._send_command(cmd)
        print("Transfer aborted")

    async def backup_firmware(self) -> Optional[bytes]:
        """Download current firmware from device."""
        print("Requesting firmware backup...")

        # Request backup
        cmd = bytes([OTA_PREFIX, OtaCommand.BACKUP_REQ])
        response = await self._send_command(cmd)

        if response is None:
            print("Backup request timeout")
            return None

        if len(response) < 6:
            print(f"Invalid backup response: {response.hex()}")
            return None

        resp_code = response[1]
        if resp_code != OtaResponse.READY:
            print(f"Backup not supported or error: code={resp_code}")
            return None

        # Parse backup info: prefix(1), code(1), size(4), total_blocks(2)
        backup_size = struct.unpack('<I', response[2:6])[0]
        total_blocks = struct.unpack('<H', response[6:8])[0] if len(response) >= 8 else 0

        if backup_size == 0:
            print("No firmware to backup (size=0)")
            return None

        print(f"Backing up {backup_size} bytes ({total_blocks} blocks)...")

        firmware_data = bytearray()
        block_num = 0

        with tqdm(total=backup_size, desc="Downloading", unit="B", unit_scale=True) as pbar:
            while len(firmware_data) < backup_size:
                # Acknowledge and request next block
                ack_cmd = struct.pack('<BBH', OTA_PREFIX, OtaCommand.BACKUP_ACK, block_num)
                response = await self._send_command(ack_cmd)

                if response is None:
                    print(f"\nBackup timeout at block {block_num}")
                    return None

                resp_code = response[1]

                if resp_code == OtaResponse.BACKUP_DATA:
                    # Parse block: prefix(1), code(1), block_num(2), crc16(2), data(...)
                    if len(response) < 6:
                        print(f"\nBackup response too short: {len(response)} bytes")
                        return None

                    recv_block = struct.unpack('<H', response[2:4])[0]
                    recv_crc = struct.unpack('<H', response[4:6])[0]
                    block_data = response[6:]

                    if recv_block != block_num:
                        print(f"\nSequence error: expected {block_num}, got {recv_block}")
                        return None

                    # Verify CRC16
                    calc_crc = crc16_ccitt(block_data)
                    if calc_crc != recv_crc:
                        print(f"\nCRC error on block {block_num}: expected 0x{recv_crc:04X}, got 0x{calc_crc:04X}")
                        return None

                    firmware_data.extend(block_data)
                    pbar.update(len(block_data))
                    block_num += 1

                elif resp_code == OtaResponse.BACKUP_DONE:
                    break
                else:
                    print(f"\nUnexpected response: code={resp_code}")
                    return None

        # Trim to actual size
        firmware_data = bytes(firmware_data[:backup_size])
        print(f"Backup complete: {len(firmware_data)} bytes")

        return firmware_data

    async def upload_firmware(
        self,
        firmware: bytes,
        version: str,
        resume_from: int = 0
    ) -> bool:
        """Upload firmware to device."""
        firmware_size = len(firmware)
        firmware_crc = crc32(firmware)
        total_blocks = (firmware_size + OTA_DATA_BLOCK_SIZE - 1) // OTA_DATA_BLOCK_SIZE

        print(f"\nFirmware: {firmware_size} bytes, {total_blocks} blocks")
        print(f"CRC32: 0x{firmware_crc:08X}")
        print(f"Version: {version}")

        # Initialize transfer
        if not await self.init_transfer(firmware_size, firmware_crc, version):
            return False

        # Send blocks
        start_time = time.time()
        bytes_sent = resume_from * OTA_DATA_BLOCK_SIZE

        # Block numbering is 1-based (1 to total_blocks)
        # This matches firmware expectation where last_block starts at 0
        # and expected_block = last_block + 1
        start_block = resume_from + 1 if resume_from > 0 else 1

        with tqdm(
            total=total_blocks,
            initial=resume_from,
            desc="Uploading",
            unit="blk"
        ) as pbar:
            for block_num in range(start_block, total_blocks + 1):
                # Extract block data (block 1 = bytes 0-223, block 2 = bytes 224-447, etc.)
                start = (block_num - 1) * OTA_DATA_BLOCK_SIZE
                end = min(start + OTA_DATA_BLOCK_SIZE, firmware_size)
                block_data = firmware[start:end]

                if not await self.send_block(block_num, block_data):
                    print(f"\nFailed at block {block_num}")
                    print(f"Resume with: --resume {block_num - 1}")
                    return False

                bytes_sent = end
                pbar.update(1)

                # Update speed estimate
                elapsed = time.time() - start_time
                if elapsed > 0:
                    speed = bytes_sent / elapsed
                    remaining = (firmware_size - bytes_sent) / speed if speed > 0 else 0
                    pbar.set_postfix({
                        'speed': f'{speed/1024:.1f}KB/s',
                        'eta': f'{remaining:.0f}s'
                    })

        elapsed = time.time() - start_time
        speed = firmware_size / elapsed if elapsed > 0 else 0
        print(f"\nTransfer complete in {elapsed:.1f}s ({speed/1024:.1f} KB/s)")

        # Validate
        print("Validating firmware...")
        valid, _ = await self.validate()

        return valid


async def main_upload(args):
    """Main upload routine."""
    uploader = OtaUploader(args.address)

    if not await uploader.connect():
        return 1

    try:
        # Load firmware
        firmware, version = load_firmware(args.firmware)
        print(f"Loaded: {args.firmware}")
        print(f"Size: {len(firmware)} bytes")
        print(f"Detected version: {version}")

        # Backup current firmware first (unless --no-backup)
        if not args.no_backup:
            backup_data = await uploader.backup_firmware()
            if backup_data:
                # Save backup
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                backup_path = f"{timestamp}_firmware.hex.bak"
                with open(backup_path, 'wb') as f:
                    f.write(backup_data)
                print(f"Backup saved: {backup_path}")
            else:
                print("Warning: Backup failed or not supported")
                if not args.force:
                    confirm = input("Continue without backup? [y/N]: ")
                    if confirm.lower() != 'y':
                        print("Aborted")
                        return 1

        # Upload
        resume_from = args.resume if hasattr(args, 'resume') else 0
        if await uploader.upload_firmware(firmware, version, resume_from):
            print("\nOTA update successful!")
            return 0
        else:
            print("\nOTA update failed!")
            return 1

    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    except ImportError as e:
        print(f"Error: {e}")
        return 1
    finally:
        await uploader.disconnect()


async def main_backup(args):
    """Main backup routine."""
    uploader = OtaUploader(args.address)

    if not await uploader.connect():
        return 1

    try:
        backup_data = await uploader.backup_firmware()
        if backup_data:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            backup_path = f"{timestamp}_firmware.hex.bak"
            with open(backup_path, 'wb') as f:
                f.write(backup_data)
            print(f"Backup saved: {backup_path}")
            print(f"Size: {len(backup_data)} bytes")
            print(f"CRC32: 0x{crc32(backup_data):08X}")
            return 0
        else:
            print("Backup failed!")
            return 1
    finally:
        await uploader.disconnect()


async def main_status(args):
    """Query device status."""
    uploader = OtaUploader(args.address)

    if not await uploader.connect():
        return 1

    try:
        status = await uploader.query_status()
        if status:
            print("\nOTA Status:")
            for key, value in status.items():
                if key == 'expected_crc':
                    print(f"  {key}: 0x{value:08X}")
                else:
                    print(f"  {key}: {value}")
            return 0
        else:
            print("Failed to query status")
            return 1
    finally:
        await uploader.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description="OTA Firmware Upload Tool for Kosmos Proxi Robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python ota_upload.py firmware.hex              # Upload with backup
  python ota_upload.py firmware.hex --no-backup  # Upload without backup
  python ota_upload.py --backup                  # Only backup current firmware
  python ota_upload.py --status                  # Query OTA status
  python ota_upload.py firmware.hex --resume 50  # Resume from block 50
        """
    )

    parser.add_argument(
        'firmware',
        nargs='?',
        default=None,
        help='Firmware file to upload (.hex or .bin)'
    )

    parser.add_argument(
        '-a', '--address',
        type=str,
        default=None,
        help='BLE device address (skip scanning)'
    )

    parser.add_argument(
        '--no-backup',
        action='store_true',
        help='Skip backup of current firmware'
    )

    parser.add_argument(
        '--backup',
        action='store_true',
        help='Only backup current firmware (no upload)'
    )

    parser.add_argument(
        '--status',
        action='store_true',
        help='Query OTA status'
    )

    parser.add_argument(
        '--resume',
        type=int,
        default=0,
        metavar='BLOCK',
        help='Resume upload from specified block number'
    )

    parser.add_argument(
        '-f', '--force',
        action='store_true',
        help='Continue even if backup fails'
    )

    args = parser.parse_args()

    # Determine mode
    if args.status:
        return asyncio.run(main_status(args))
    elif args.backup:
        return asyncio.run(main_backup(args))
    elif args.firmware:
        return asyncio.run(main_upload(args))
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
