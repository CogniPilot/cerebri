# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

"""
ESC Passthrough USB Protocol Test Configuration

Provides fixtures for testing MSP + 4-way interface protocol over USB CDC.
The device console is monitored by twister on /dev/ttyUSB0, while these
tests communicate over USB CDC on /dev/ttyACM0.
"""

import logging
import time

import pytest

logger = logging.getLogger(__name__)


def pytest_addoption(parser) -> None:
    """Add local parser options to pytest."""
    parser.addoption('--usb-port', default='/dev/ttyACM0',
                     help='USB CDC serial port for MSP/4-way protocol')


class MSPProtocol:
    """MSP (MultiWii Serial Protocol) v1 implementation."""

    # MSP commands
    API_VERSION = 1
    FC_VARIANT = 2
    FC_VERSION = 3
    BOARD_INFO = 4
    BUILD_INFO = 5
    MOTOR = 104
    BATTERY_STATE = 130
    MOTOR_CONFIG = 131
    SET_4WAY_IF = 245

    def __init__(self, serial_port):
        self.ser = serial_port

    def send(self, cmd: int, payload: bytes = b"") -> bytes:
        """Send MSP command and receive response."""
        # Build MSP packet: $M< len cmd [payload] checksum
        plen = len(payload)
        cksum = plen ^ cmd
        for b in payload:
            cksum ^= b

        packet = b"$M<" + bytes([plen, cmd]) + payload + bytes([cksum])
        self.ser.write(packet)
        self.ser.flush()

        # Read response: $M> len cmd [payload] checksum
        header = self.ser.read(5)
        if len(header) < 5:
            raise TimeoutError(f"MSP timeout (got {len(header)} bytes)")
        if header[:3] != b"$M>":
            raise ValueError(f"Invalid MSP header: {header.hex()}")

        resp_len = header[3]
        resp_cmd = header[4]
        resp_data = self.ser.read(resp_len) if resp_len > 0 else b""
        resp_cksum = self.ser.read(1)

        if len(resp_data) < resp_len:
            raise TimeoutError(f"MSP payload truncated: expected {resp_len}, got {len(resp_data)}")

        # Verify checksum
        calc_cksum = resp_len ^ resp_cmd
        for b in resp_data:
            calc_cksum ^= b
        if resp_cksum and resp_cksum[0] != calc_cksum:
            raise ValueError(f"MSP checksum mismatch: {resp_cksum[0]} != {calc_cksum}")

        return resp_data


class FourWayProtocol:
    """4-Way Interface Protocol for ESC communication."""

    # Commands
    INTERFACE_TEST_ALIVE = 0x30
    PROTOCOL_GET_VERSION = 0x31
    INTERFACE_GET_NAME = 0x32
    INTERFACE_GET_VERSION = 0x33
    INTERFACE_EXIT = 0x34
    DEVICE_RESET = 0x35
    DEVICE_INIT_FLASH = 0x37
    DEVICE_PAGE_ERASE = 0x39
    DEVICE_READ = 0x3A
    DEVICE_WRITE = 0x3B
    DEVICE_READ_EEPROM = 0x3D
    DEVICE_WRITE_EEPROM = 0x3E
    INTERFACE_SET_MODE = 0x3F

    # ACK codes
    ACK_OK = 0x00
    ACK_UNKNOWN_ERROR = 0x01
    ACK_INVALID_CMD = 0x02
    ACK_INVALID_CRC = 0x03
    ACK_VERIFY_ERROR = 0x04
    ACK_INVALID_CHANNEL = 0x08
    ACK_GENERAL_ERROR = 0x0F

    def __init__(self, serial_port):
        self.ser = serial_port

    @staticmethod
    def crc16(data: bytes) -> int:
        """CRC-16 CCITT (polynomial 0x1021, MSB first)."""
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def send(self, cmd: int, addr: int = 0, params: bytes = b"") -> tuple:
        """Send 4-way command and receive response.

        Returns: (ack_code, response_data)
        """
        # Build packet: [0x2F] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [crc_hi] [crc_lo]
        param_count = len(params) if len(params) < 256 else 0
        packet = bytes([0x2F, cmd, addr >> 8, addr & 0xFF, param_count]) + params
        crc = self.crc16(packet)
        packet += bytes([crc >> 8, crc & 0xFF])

        self.ser.write(packet)
        self.ser.flush()

        # Read response: [0x2E] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [ack] [crc_hi] [crc_lo]
        header = self.ser.read(5)
        if len(header) < 5:
            raise TimeoutError(f"4-way timeout (got {len(header)} bytes)")
        if header[0] != 0x2E:
            raise ValueError(f"Invalid 4-way marker: 0x{header[0]:02X}")

        resp_param_count = header[4]
        actual_params = 256 if resp_param_count == 0 else resp_param_count

        # Read params + ack + crc
        remaining = self.ser.read(actual_params + 3)
        if len(remaining) < actual_params + 3:
            raise TimeoutError(f"4-way truncated: expected {actual_params + 3}, got {len(remaining)}")

        resp_params = remaining[:actual_params]
        ack = remaining[actual_params]
        rx_crc = (remaining[actual_params + 1] << 8) | remaining[actual_params + 2]

        # Verify CRC
        crc_data = header + resp_params + bytes([ack])
        calc_crc = self.crc16(crc_data)
        if rx_crc != calc_crc:
            raise ValueError(f"4-way CRC mismatch: 0x{rx_crc:04X} != 0x{calc_crc:04X}")

        return ack, resp_params


@pytest.fixture(scope='session')
def usb_port(request) -> str:
    """Return the USB CDC port path."""
    return request.config.getoption('--usb-port')


@pytest.fixture(scope='session')
def usb_serial(usb_port: str):
    """Open USB CDC serial port for MSP/4-way protocol.

    Waits for USB enumeration after device flash.
    """
    import serial

    # Wait for USB CDC to enumerate after flash
    logger.info(f"Waiting for USB CDC enumeration on {usb_port}...")
    for attempt in range(10):
        try:
            ser = serial.Serial(usb_port, 115200, timeout=2.0)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            logger.info(f"Connected to {usb_port}")
            yield ser
            ser.close()
            return
        except serial.SerialException:
            time.sleep(1)

    pytest.fail(f"USB CDC port {usb_port} did not enumerate after 10 seconds")


@pytest.fixture(scope='session')
def msp(usb_serial) -> MSPProtocol:
    """Return MSP protocol handler."""
    return MSPProtocol(usb_serial)


@pytest.fixture(scope='session')
def fourway(usb_serial) -> FourWayProtocol:
    """Return 4-way protocol handler."""
    return FourWayProtocol(usb_serial)
