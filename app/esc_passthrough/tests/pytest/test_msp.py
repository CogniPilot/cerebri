# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

"""
MSP Protocol Tests

Tests the MultiWii Serial Protocol implementation over USB CDC.
These commands are used by ESC configurators to identify the flight controller.
"""

import logging

from conftest import MSPProtocol

logger = logging.getLogger(__name__)


class TestMSPIdentification:
    """Test MSP identification commands."""

    def test_api_version(self, msp: MSPProtocol):
        """Test MSP_API_VERSION returns valid version."""
        resp = msp.send(MSPProtocol.API_VERSION)
        assert len(resp) >= 4, "API version should return at least 4 bytes"
        # Bytes: [protocol_version, api_major, api_minor, api_patch]
        logger.info(f"API Version: {resp[1]}.{resp[2]}.{resp[3]}")
        assert resp[1] >= 1, "API major version should be >= 1"

    def test_fc_variant(self, msp: MSPProtocol):
        """Test MSP_FC_VARIANT returns BTFL (Betaflight)."""
        resp = msp.send(MSPProtocol.FC_VARIANT)
        assert len(resp) >= 4, "FC variant should return at least 4 bytes"
        variant = resp[:4].decode('ascii', errors='replace')
        logger.info(f"FC Variant: {variant}")
        assert variant == "BTFL", f"Expected BTFL, got {variant}"

    def test_fc_version(self, msp: MSPProtocol):
        """Test MSP_FC_VERSION returns valid version."""
        resp = msp.send(MSPProtocol.FC_VERSION)
        assert len(resp) >= 3, "FC version should return at least 3 bytes"
        logger.info(f"FC Version: {resp[0]}.{resp[1]}.{resp[2]}")
        assert resp[0] >= 4, "FC major version should be >= 4"

    def test_board_info(self, msp: MSPProtocol):
        """Test MSP_BOARD_INFO returns board identifier."""
        resp = msp.send(MSPProtocol.BOARD_INFO)
        assert len(resp) >= 4, "Board info should return at least 4 bytes"
        board = resp[:4].decode('ascii', errors='replace')
        logger.info(f"Board: {board}")
        assert board == "VMU1", f"Expected VMU1, got {board}"


class TestMSPMotorConfig:
    """Test MSP motor configuration commands."""

    def test_motor_config(self, msp: MSPProtocol):
        """Test MSP_MOTOR_CONFIG returns motor count."""
        resp = msp.send(MSPProtocol.MOTOR_CONFIG)
        assert len(resp) >= 7, "Motor config should return at least 7 bytes"
        motor_count = resp[6]
        logger.info(f"Motor count: {motor_count}")
        assert motor_count >= 1, "Should have at least 1 motor"

    def test_motor_values(self, msp: MSPProtocol):
        """Test MSP_MOTOR returns motor values."""
        resp = msp.send(MSPProtocol.MOTOR)
        # 8 motors x 2 bytes = 16 bytes
        assert len(resp) >= 2, "Motor values should return at least 2 bytes"
        logger.info(f"Motor data: {len(resp)} bytes")


class TestMSP4WayEntry:
    """Test entering 4-way interface mode via MSP."""

    def test_enter_4way_mode(self, msp: MSPProtocol):
        """Test MSP_SET_4WAY_IF enters 4-way mode and returns ESC count."""
        resp = msp.send(MSPProtocol.SET_4WAY_IF)
        assert len(resp) >= 1, "SET_4WAY_IF should return at least 1 byte"
        esc_count = resp[0]
        logger.info(f"ESC count: {esc_count}")
        assert esc_count >= 1, "Should have at least 1 ESC"
