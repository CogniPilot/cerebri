# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

"""
4-Way Interface Protocol Tests

Tests the BLHeli/AM32 4-way interface protocol over USB CDC.
These commands are used by ESC configurators to read/write ESC settings.

Note: Device must be in 4-way mode (via MSP_SET_4WAY_IF) before these tests run.
"""

import logging

import pytest
from conftest import MSPProtocol, FourWayProtocol

logger = logging.getLogger(__name__)


@pytest.fixture(scope='module')
def fourway_mode(msp: MSPProtocol, fourway: FourWayProtocol):
    """Ensure device is in 4-way mode before running tests."""
    # Enter 4-way mode via MSP
    resp = msp.send(MSPProtocol.SET_4WAY_IF)
    esc_count = resp[0] if resp else 0
    logger.info(f"Entered 4-way mode, ESC count: {esc_count}")
    assert esc_count >= 1, "Failed to enter 4-way mode"

    yield fourway

    # Exit 4-way mode after tests
    try:
        ack, _ = fourway.send(FourWayProtocol.INTERFACE_EXIT)
        logger.info(f"Exited 4-way mode, ACK: 0x{ack:02X}")
    except Exception as e:
        logger.warning(f"Failed to exit 4-way mode: {e}")


class TestFourWayInterface:
    """Test 4-way interface commands."""

    def test_interface_test_alive(self, fourway_mode: FourWayProtocol):
        """Test INTERFACE_TEST_ALIVE command."""
        ack, data = fourway_mode.send(FourWayProtocol.INTERFACE_TEST_ALIVE)
        logger.info(f"TEST_ALIVE ACK: 0x{ack:02X}")
        assert ack == FourWayProtocol.ACK_OK, f"Expected ACK_OK, got 0x{ack:02X}"

    def test_protocol_get_version(self, fourway_mode: FourWayProtocol):
        """Test PROTOCOL_GET_VERSION command."""
        ack, data = fourway_mode.send(FourWayProtocol.PROTOCOL_GET_VERSION)
        assert ack == FourWayProtocol.ACK_OK, f"Expected ACK_OK, got 0x{ack:02X}"
        assert len(data) >= 1, "Should return protocol version"
        logger.info(f"Protocol version: {data[0]}")

    def test_interface_get_name(self, fourway_mode: FourWayProtocol):
        """Test INTERFACE_GET_NAME command."""
        ack, data = fourway_mode.send(FourWayProtocol.INTERFACE_GET_NAME)
        assert ack == FourWayProtocol.ACK_OK, f"Expected ACK_OK, got 0x{ack:02X}"
        name = data.decode('ascii', errors='replace').rstrip('\x00')
        logger.info(f"Interface name: {name}")
        assert len(name) > 0, "Interface name should not be empty"

    def test_interface_get_version(self, fourway_mode: FourWayProtocol):
        """Test INTERFACE_GET_VERSION command."""
        ack, data = fourway_mode.send(FourWayProtocol.INTERFACE_GET_VERSION)
        assert ack == FourWayProtocol.ACK_OK, f"Expected ACK_OK, got 0x{ack:02X}"
        assert len(data) >= 2, "Should return 2-byte version"
        version = (data[0] << 8) | data[1]
        logger.info(f"Interface version: {version}")

    def test_interface_set_mode(self, fourway_mode: FourWayProtocol):
        """Test INTERFACE_SET_MODE command to select ESC 0."""
        ack, _ = fourway_mode.send(FourWayProtocol.INTERFACE_SET_MODE, params=bytes([0]))
        logger.info(f"SET_MODE(0) ACK: 0x{ack:02X}")
        assert ack == FourWayProtocol.ACK_OK, f"Expected ACK_OK, got 0x{ack:02X}"

    def test_interface_set_mode_invalid(self, fourway_mode: FourWayProtocol):
        """Test INTERFACE_SET_MODE with invalid channel returns error."""
        ack, _ = fourway_mode.send(FourWayProtocol.INTERFACE_SET_MODE, params=bytes([99]))
        logger.info(f"SET_MODE(99) ACK: 0x{ack:02X}")
        assert ack == FourWayProtocol.ACK_INVALID_CHANNEL, \
            f"Expected ACK_INVALID_CHANNEL (0x08), got 0x{ack:02X}"


class TestFourWayESC:
    """Test 4-way ESC communication (requires ESC in bootloader mode)."""

    @pytest.mark.skipif(True, reason="Requires ESC powered in bootloader mode")
    def test_device_init_flash(self, fourway_mode: FourWayProtocol):
        """Test DEVICE_INIT_FLASH queries ESC bootloader.

        This test requires an AM32 ESC connected and powered with the signal
        held HIGH during power-on to enter bootloader mode.
        """
        # Select ESC 0
        fourway_mode.send(FourWayProtocol.INTERFACE_SET_MODE, params=bytes([0]))

        # Query bootloader
        ack, data = fourway_mode.send(FourWayProtocol.DEVICE_INIT_FLASH, params=bytes([0]))
        logger.info(f"INIT_FLASH ACK: 0x{ack:02X}, data: {data.hex() if data else 'none'}")

        if ack == FourWayProtocol.ACK_OK:
            assert len(data) >= 4, "Should return 4 bytes of device info"
            sig_lo = data[0]
            sig_hi = data[1]
            sig = (sig_hi << 8) | sig_lo
            mode = data[3]
            logger.info(f"ESC signature: 0x{sig:04X}, mode: {mode}")
        else:
            pytest.skip("ESC not responding - ensure bootloader mode")
