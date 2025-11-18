"""
Test that reproduces the exact AM32.ca website flow.

This test simulates what happens when you connect to AM32.ca configurator:
1. MSP API_VERSION, FC_VARIANT, BATTERY_STATE, MOTOR_CONFIG queries
2. MSP SET_4WAY_IF to enter 4-way mode
3. 4-WAY DeviceInitFlash to query ESC bootloader

The ESC bootloader query will fail with ACK_GENERAL_ERROR (0x0F) if:
- No ESC is connected
- ESC wasn't powered with signal HIGH (bootloader entry condition)
- ESC is running normal firmware (not in bootloader mode)
"""

import pytest
import serial
import time


def crc16_fourway(data: bytes) -> int:
    """CRC16 for 4-way protocol (polynomial 0x1021, MSB first)"""
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


class MSPProtocol:
    """MSP v1 protocol implementation"""

    def __init__(self, port: serial.Serial):
        self.port = port

    def send_command(self, cmd: int, payload: bytes = b'') -> bytes:
        """Send MSP command and return response payload"""
        # Build request: $M< + size + cmd + payload + checksum
        size = len(payload)
        checksum = size ^ cmd
        for b in payload:
            checksum ^= b

        msg = b'$M<' + bytes([size, cmd]) + payload + bytes([checksum])
        self.port.write(msg)

        # Read response
        time.sleep(0.05)
        resp = self.port.read(256)

        if len(resp) < 6:
            raise TimeoutError(f"Short MSP response: {len(resp)} bytes")

        # Parse response: $M> + size + cmd + data + checksum
        if resp[0:3] != b'$M>':
            raise ValueError(f"Invalid MSP header: {resp[0:3].hex()}")

        resp_size = resp[3]
        data = resp[5:5+resp_size]

        return data


class FourWayProtocol:
    """4-Way Interface protocol implementation"""

    # Commands
    CMD_INTERFACE_TEST_ALIVE = 0x30
    CMD_PROTOCOL_GET_VERSION = 0x31
    CMD_INTERFACE_GET_NAME = 0x32
    CMD_INTERFACE_GET_VERSION = 0x33
    CMD_INTERFACE_EXIT = 0x34
    CMD_DEVICE_RESET = 0x35
    CMD_DEVICE_INIT_FLASH = 0x37
    CMD_DEVICE_ERASE_ALL = 0x38
    CMD_DEVICE_PAGE_ERASE = 0x39
    CMD_DEVICE_READ = 0x3A
    CMD_DEVICE_WRITE = 0x3B
    CMD_DEVICE_READ_EEPROM = 0x3D
    CMD_DEVICE_WRITE_EEPROM = 0x3E
    CMD_INTERFACE_SET_MODE = 0x3F

    # Response codes
    ACK_OK = 0x00
    ACK_UNKNOWN_ERROR = 0x01
    ACK_INVALID_CMD = 0x02
    ACK_INVALID_CRC = 0x03
    ACK_VERIFY_ERROR = 0x04
    ACK_INVALID_CHANNEL = 0x08
    ACK_GENERAL_ERROR = 0x0F

    def __init__(self, port: serial.Serial):
        self.port = port

    def send_command(self, cmd: int, addr: int = 0, params: bytes = b'', timeout: float = 2.0) -> tuple:
        """
        Send 4-way command and return (ack_code, response_data)

        Request format: [0x2F] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [crc_hi] [crc_lo]
        Response format: [0x2E] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [ack] [crc_hi] [crc_lo]
        """
        # Build packet
        param_count = len(params) if len(params) < 256 else 0
        pkt = bytes([0x2F, cmd, addr >> 8, addr & 0xFF, param_count]) + params
        crc = crc16_fourway(pkt)
        pkt += bytes([crc >> 8, crc & 0xFF])

        # Send
        self.port.reset_input_buffer()
        self.port.write(pkt)

        # Read response with timeout
        self.port.timeout = timeout
        resp = self.port.read(270)

        if len(resp) < 8:
            raise TimeoutError(f"Short 4-way response: {len(resp)} bytes, data: {resp.hex()}")

        # Parse response
        if resp[0] != 0x2E:
            raise ValueError(f"Invalid 4-way response marker: 0x{resp[0]:02X}")

        resp_param_count = resp[4]
        actual_params = 256 if resp_param_count == 0 else resp_param_count

        # Find ACK byte (after params)
        ack_idx = 5 + actual_params
        if len(resp) < ack_idx + 3:  # ACK + 2 CRC bytes
            raise ValueError(f"Response too short for param_count={resp_param_count}")

        ack = resp[ack_idx]
        data = resp[5:5+actual_params] if actual_params > 0 else b''

        # Verify CRC
        crc_idx = ack_idx + 1
        rx_crc = (resp[crc_idx] << 8) | resp[crc_idx + 1]
        calc_crc = crc16_fourway(resp[:crc_idx])

        if rx_crc != calc_crc:
            raise ValueError(f"4-way CRC mismatch: expected 0x{calc_crc:04X}, got 0x{rx_crc:04X}")

        return (ack, data)


class TestAM32WebsiteFlow:
    """Test class that reproduces the AM32.ca website flow exactly"""

    @pytest.fixture
    def usb_port(self):
        """Find USB CDC ACM port"""
        import glob
        ports = glob.glob('/dev/ttyACM*')
        if not ports:
            pytest.skip("No USB CDC ACM device found")
        return ports[0]

    @pytest.fixture
    def serial_conn(self, usb_port):
        """Open serial connection"""
        port = serial.Serial(usb_port, 115200, timeout=1)
        time.sleep(0.5)
        port.reset_input_buffer()
        yield port
        port.close()

    def test_msp_sequence(self, serial_conn):
        """Test the MSP command sequence that AM32.ca sends first"""
        msp = MSPProtocol(serial_conn)

        # 1. API_VERSION (cmd=1)
        resp = msp.send_command(1)
        assert len(resp) == 4, f"API_VERSION: expected 4 bytes, got {len(resp)}"
        api_major = resp[1]
        api_minor = resp[2]
        print(f"API Version: {api_major}.{api_minor}")

        # 2. FC_VARIANT (cmd=2)
        resp = msp.send_command(2)
        assert len(resp) == 4, f"FC_VARIANT: expected 4 bytes, got {len(resp)}"
        variant = resp.decode('ascii', errors='replace')
        print(f"FC Variant: {variant}")
        assert variant == "BTFL", f"Expected BTFL variant, got {variant}"

        # 3. BATTERY_STATE (cmd=130)
        resp = msp.send_command(130)
        assert len(resp) >= 9, f"BATTERY_STATE: expected >=9 bytes, got {len(resp)}"
        print(f"Battery state: {len(resp)} bytes")

        # 4. MOTOR_CONFIG (cmd=131)
        resp = msp.send_command(131)
        assert len(resp) >= 7, f"MOTOR_CONFIG: expected >=7 bytes, got {len(resp)}"
        motor_count = resp[6]
        print(f"Motor count: {motor_count}")

    def test_enter_fourway_mode(self, serial_conn):
        """Test entering 4-way mode via MSP"""
        msp = MSPProtocol(serial_conn)

        # MSP_SET_4WAY_IF (cmd=245)
        resp = msp.send_command(245)
        assert len(resp) == 1, f"SET_4WAY_IF: expected 1 byte, got {len(resp)}"
        esc_count = resp[0]
        print(f"4-way mode entered, ESC count: {esc_count}")
        assert esc_count >= 1, "Should have at least 1 ESC channel"

    def test_fourway_interface_alive(self, serial_conn):
        """Test 4-way InterfaceTestAlive command"""
        msp = MSPProtocol(serial_conn)

        # Enter 4-way mode first
        msp.send_command(245)
        time.sleep(0.05)

        # Test InterfaceTestAlive
        fw = FourWayProtocol(serial_conn)
        ack, data = fw.send_command(fw.CMD_INTERFACE_TEST_ALIVE)

        print(f"InterfaceTestAlive: ACK=0x{ack:02X}")
        assert ack == fw.ACK_OK, f"Expected ACK_OK (0x00), got 0x{ack:02X}"

    def test_fourway_get_info(self, serial_conn):
        """Test 4-way interface info commands"""
        msp = MSPProtocol(serial_conn)

        # Enter 4-way mode
        msp.send_command(245)
        time.sleep(0.05)

        fw = FourWayProtocol(serial_conn)

        # Protocol version
        ack, data = fw.send_command(fw.CMD_PROTOCOL_GET_VERSION)
        assert ack == fw.ACK_OK
        print(f"Protocol version: {data[0] if data else 'N/A'}")

        # Interface name
        ack, data = fw.send_command(fw.CMD_INTERFACE_GET_NAME)
        assert ack == fw.ACK_OK
        name = data.decode('ascii', errors='replace').rstrip('\x00')
        print(f"Interface name: {name}")

        # Interface version
        ack, data = fw.send_command(fw.CMD_INTERFACE_GET_VERSION)
        assert ack == fw.ACK_OK
        if len(data) >= 2:
            version = (data[0] << 8) | data[1]
            print(f"Interface version: {version}")

    def test_fourway_device_init_flash(self, serial_conn):
        """
        Test DeviceInitFlash - this queries the ESC bootloader.

        This will return ACK_GENERAL_ERROR (0x0F) if:
        - No ESC connected
        - ESC not in bootloader mode
        - ESC wasn't powered with signal HIGH

        This is the exact failure seen in the AM32.ca log.
        """
        msp = MSPProtocol(serial_conn)

        # Enter 4-way mode
        msp.send_command(245)
        time.sleep(0.05)

        fw = FourWayProtocol(serial_conn)

        # DeviceInitFlash for ESC 0
        # Request: [0x2F] [0x37] [0x00] [0x00] [0x01] [0x00] [CRC_HI] [CRC_LO]
        #   0x37 = DEVICE_INIT_FLASH
        #   addr = 0x0000
        #   param_count = 1
        #   param[0] = 0 (ESC index 0)
        ack, data = fw.send_command(fw.CMD_DEVICE_INIT_FLASH, addr=0, params=bytes([0]), timeout=3.0)

        print(f"DeviceInitFlash: ACK=0x{ack:02X}, data={data.hex() if data else 'empty'}")

        if ack == fw.ACK_OK:
            # ESC bootloader responded!
            assert len(data) == 4, f"Expected 4 bytes of device info, got {len(data)}"
            sig_lo = data[0]
            sig_hi = data[1]
            signature = (sig_hi << 8) | sig_lo
            input_val = data[2]
            mode = data[3]
            print(f"  ESC Bootloader found!")
            print(f"  Signature: 0x{signature:04X}")
            print(f"  Input: 0x{input_val:02X}")
            print(f"  Mode: {mode} ({'ARM' if mode == 4 else 'SiLabs' if mode == 1 else 'Unknown'})")
        elif ack == fw.ACK_GENERAL_ERROR:
            # This is the expected failure when no ESC bootloader responds
            print(f"  ESC bootloader not responding (expected if no ESC connected)")
            print(f"  To fix: Power ESC with signal HIGH, then try again")
        else:
            pytest.fail(f"Unexpected ACK code: 0x{ack:02X}")

    def test_fourway_exit(self, serial_conn):
        """Test exiting 4-way mode"""
        msp = MSPProtocol(serial_conn)

        # Enter 4-way mode
        msp.send_command(245)
        time.sleep(0.05)

        fw = FourWayProtocol(serial_conn)

        # Exit 4-way mode
        ack, data = fw.send_command(fw.CMD_INTERFACE_EXIT)

        print(f"InterfaceExit: ACK=0x{ack:02X}")
        assert ack == fw.ACK_OK, f"Expected ACK_OK, got 0x{ack:02X}"

    def test_fourway_read_eeprom(self, serial_conn):
        """
        Test DeviceRead command to read EEPROM data.

        This mimics what AM32.ca does after DeviceInitFlash succeeds:
        it reads EEPROM to display current settings.
        """
        msp = MSPProtocol(serial_conn)

        # Enter 4-way mode
        msp.send_command(245)
        time.sleep(0.05)

        fw = FourWayProtocol(serial_conn)

        # First try to init ESC 0
        ack, data = fw.send_command(fw.CMD_DEVICE_INIT_FLASH, addr=0, params=bytes([0]), timeout=3.0)

        if ack != fw.ACK_OK:
            pytest.skip("ESC not responding - skip read test")

        # Read 256 bytes from EEPROM (0x7C00)
        # AM32 uses count=0 to mean 256 bytes
        print("Reading EEPROM from 0x7C00...")
        ack, data = fw.send_command(fw.CMD_DEVICE_READ, addr=0x7C00, params=bytes([0]), timeout=2.0)

        print(f"DeviceRead: ACK=0x{ack:02X}, got {len(data)} bytes")

        if ack == fw.ACK_OK:
            assert len(data) == 256, f"Expected 256 bytes, got {len(data)}"
            print(f"  First 16 bytes: {data[:16].hex()}")

        # Exit
        fw.send_command(fw.CMD_INTERFACE_EXIT)

    def test_fourway_write_settings(self, serial_conn):
        """
        Test DeviceWrite command - this tests the bug fix for write ACK responses.

        AM32.ca sends DeviceWrite with EEPROM data, expects ACK_OK response.
        The bug was param_count=0 meaning 256, causing timeout waiting for data.

        FIX: Empty responses now use param_count=1 with a dummy byte.
        """
        msp = MSPProtocol(serial_conn)

        # Enter 4-way mode
        msp.send_command(245)
        time.sleep(0.05)

        fw = FourWayProtocol(serial_conn)

        # First init ESC 0
        ack, data = fw.send_command(fw.CMD_DEVICE_INIT_FLASH, addr=0, params=bytes([0]), timeout=3.0)

        if ack != fw.ACK_OK:
            pytest.skip("ESC not responding - skip write test")

        # Read current EEPROM first
        ack, original_data = fw.send_command(fw.CMD_DEVICE_READ, addr=0x7C00, params=bytes([0]), timeout=2.0)

        if ack != fw.ACK_OK or len(original_data) != 256:
            pytest.skip("Could not read EEPROM")

        print(f"Read {len(original_data)} bytes from EEPROM")

        # Write back the same data (safe - no actual change)
        # AM32.ca writes 184 bytes to EEPROM at 0x7C00
        write_data = original_data[:184]

        print(f"Writing {len(write_data)} bytes to 0x7C00...")
        ack, resp_data = fw.send_command(fw.CMD_DEVICE_WRITE, addr=0x7C00, params=write_data, timeout=2.0)

        print(f"DeviceWrite: ACK=0x{ack:02X}")

        # This is the critical test - the write ACK must be accepted
        assert ack == fw.ACK_OK, f"DeviceWrite should return ACK_OK (0x00), got 0x{ack:02X}"
        print("  PASS: Write ACK received correctly!")

        # Exit
        fw.send_command(fw.CMD_INTERFACE_EXIT)

    def test_full_am32_flow(self, serial_conn):
        """
        Complete AM32.ca website flow in one test.

        This reproduces exactly what happens when you connect am32.ca to the device:
        1. MSP queries to identify the flight controller
        2. Enter 4-way mode
        3. Query ESC bootloader (DeviceInitFlash)
        4. Exit 4-way mode (on failure or completion)
        """
        msp = MSPProtocol(serial_conn)

        print("\n=== AM32.ca Website Flow Simulation ===\n")

        # Step 1: MSP identification sequence
        print("Step 1: MSP Identification")

        resp = msp.send_command(1)  # API_VERSION
        print(f"  API Version: {resp[1]}.{resp[2]}.{resp[3]}")

        resp = msp.send_command(2)  # FC_VARIANT
        print(f"  FC Variant: {resp.decode('ascii')}")

        resp = msp.send_command(130)  # BATTERY_STATE
        print(f"  Battery State: {len(resp)} bytes")

        resp = msp.send_command(131)  # MOTOR_CONFIG
        motor_count = resp[6]
        print(f"  Motor Config: {motor_count} motors")

        # Step 2: Enter 4-way mode
        print("\nStep 2: Enter 4-Way Mode")
        resp = msp.send_command(245)  # SET_4WAY_IF
        esc_count = resp[0]
        print(f"  4-Way mode active, {esc_count} ESC channels")
        time.sleep(0.05)

        # Step 3: Query ESC bootloader
        print("\nStep 3: Query ESC Bootloaders")
        fw = FourWayProtocol(serial_conn)

        for esc_idx in range(esc_count):
            print(f"\n  ESC {esc_idx + 1}:")
            ack, data = fw.send_command(fw.CMD_DEVICE_INIT_FLASH, addr=0, params=bytes([esc_idx]), timeout=3.0)

            if ack == fw.ACK_OK:
                sig = (data[1] << 8) | data[0]
                print(f"    FOUND! Signature: 0x{sig:04X}, Mode: {data[3]}")
            elif ack == fw.ACK_GENERAL_ERROR:
                print(f"    No response (ESC not in bootloader mode)")
            elif ack == fw.ACK_INVALID_CHANNEL:
                print(f"    Invalid channel")
            else:
                print(f"    Error: ACK=0x{ack:02X}")

        # Step 4: Exit 4-way mode
        print("\nStep 4: Exit 4-Way Mode")
        ack, _ = fw.send_command(fw.CMD_INTERFACE_EXIT)
        print(f"  Exit: {'OK' if ack == fw.ACK_OK else f'Error 0x{ack:02X}'}")

        print("\n=== Flow Complete ===\n")


if __name__ == '__main__':
    # Run directly for quick testing
    import sys

    port_path = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    print(f"Testing with port: {port_path}")

    port = serial.Serial(port_path, 115200, timeout=1)
    time.sleep(0.5)
    port.reset_input_buffer()

    test = TestAM32WebsiteFlow()

    # Run the full flow test
    class FakeSerial:
        def __init__(self, p):
            self.p = p

    # Run manually
    msp = MSPProtocol(port)

    print("\n=== Quick AM32 Flow Test ===\n")

    # MSP tests
    resp = msp.send_command(1)
    print(f"API Version: {resp[1]}.{resp[2]}")

    resp = msp.send_command(2)
    print(f"FC Variant: {resp.decode('ascii')}")

    # Enter 4-way
    resp = msp.send_command(245)
    print(f"4-Way mode: {resp[0]} ESCs")
    time.sleep(0.05)

    # DeviceInitFlash
    fw = FourWayProtocol(port)
    ack, data = fw.send_command(fw.CMD_DEVICE_INIT_FLASH, addr=0, params=bytes([0]), timeout=3.0)

    print(f"\nDeviceInitFlash result:")
    print(f"  ACK: 0x{ack:02X} ({['OK', 'UNKNOWN_ERR', 'INVALID_CMD', 'INVALID_CRC', 'VERIFY_ERR', '', '', '', 'INVALID_CH', '', '', '', '', '', '', 'GENERAL_ERR'][ack] if ack < 16 else 'UNKNOWN'})")
    if ack == 0 and data:
        sig = (data[1] << 8) | data[0]
        print(f"  Signature: 0x{sig:04X}")
        print(f"  Mode: {data[3]}")
    elif ack == 0x0F:
        print(f"  ESC bootloader not responding")
        print(f"  Make sure ESC was powered with signal HIGH")

    # Exit 4-way
    ack, _ = fw.send_command(fw.CMD_INTERFACE_EXIT)
    print(f"\n4-Way Exit: {'OK' if ack == 0 else f'Error 0x{ack:02X}'}")

    port.close()
