.. _esc_passthrough:

ESC Passthrough
###############

Overview
********

This application implements ESC passthrough communication for configuring AM32
ESCs through the CogniPilot VMU. It enables web-based configuration tools like
`AM32.ca <https://am32.ca>`_ to read and write ESC settings via USB.

The passthrough bridges three protocols:

1. **USB CDC-ACM** - Host computer connection
2. **MSP (MultiWii Serial Protocol)** - Flight controller communication
3. **4-Way Interface** - BLHeli/AM32 ESC bootloader protocol
4. **Single-wire serial** - Direct ESC bootloader communication at 19200 baud

Supported Configurators
***********************

* `AM32 Configurator <https://am32.ca>`_ - Primary target
* `ESC Configurator <https://esc-configurator.com>`_

Requirements
************

* Board with GPIO pins connected to ESC signal lines
* AM32-compatible ESC(s)
* USB connection for CDC-ACM virtual serial port

Building and Running
********************

Build the application for your board:

.. code-block:: console

   west build -b vmu_rt1170/mimxrt1176/cm7 app/esc_passthrough

Flash to the device:

.. code-block:: console

   west flash

Usage
*****

.. warning::

   **REMOVE ALL PROPELLERS** before configuring ESCs. Motors may spin
   unexpectedly during configuration, which can cause serious injury.

Step-by-Step Configuration
==========================

1. **Remove propellers** from all motors

2. **Connect VMU to computer via USB-C**

   - The USB connection provides both power to the VMU and communication
   - Do NOT connect battery yet

3. **Wait for boot message**

   Open a serial terminal to the debug port (``/dev/ttyUSB0``) and wait for:

   .. code-block:: none

      *** Booting Zephyr OS ***
      ESC Passthrough - MSP + 4-Way Interface
      ESC 1: GPIO1.23 = HIGH
      ESC 2: GPIO1.25 = HIGH
      ESC 3: GPIO1.27 = HIGH
      ESC 4: GPIO1.6 = HIGH
      *** ALL ESC SIGNALS HIGH - PLUG IN BATTERY NOW ***

4. **Connect battery AFTER seeing the boot message**

   - The ESCs must see HIGH signal when they power up to enter bootloader mode
   - If you connect the battery too early, the ESCs will boot normally and
     won't be accessible for configuration

5. **Open AM32 Configurator**

   - Navigate to https://am32.ca in Chrome or Edge (WebSerial required)
   - Click "Connect" and select the USB serial port
   - On Linux: typically ``/dev/ttyACM0``
   - On Windows: COMx port
   - On macOS: ``/dev/tty.usbmodemXXXX``

6. **Configure your ESCs**

   - The configurator will detect connected ESCs
   - Read settings, modify as needed, and write changes
   - Disconnect battery and reconnect to apply new settings

Shell Commands
==============

The application provides shell commands for debugging:

.. code-block:: console

   uart:~$ esc status      # Show ESC GPIO states
   uart:~$ esc query       # Query ESC bootloader directly
   uart:~$ esc eeprom      # Read EEPROM from ESC
   uart:~$ esc 4way        # Show 4-way interface status

Protocol Details
****************

The implementation follows these specifications:

* **AM32 Bootloader**: https://github.com/am32-firmware/AM32-Bootloader
* **AM32 Configurator**: https://github.com/am32-firmware/am32-configurator

Key protocol details:

* Single-wire serial at 19200 baud (fixed, per BLHeli spec)
* Bootloader CRC: polynomial 0xA001, LSB-first
* 4-Way CRC: polynomial 0x1021 (CRC16-XMODEM), MSB-first
* ``param_count=0`` means 256 bytes in 4-Way responses

See ``tests/test_protocol.c`` for comprehensive protocol documentation.

Device Tree Configuration
*************************

ESC GPIO pins are configured via device tree overlay. Example for 4 channels:

.. code-block:: devicetree

   / {
       esc_gpios: esc_gpios {
           compatible = "gpio-leds";
           status = "okay";
           pinctrl-0 = <&pinmux_esc_gpio>;
           pinctrl-names = "default";

           esc1: esc_1 {
               gpios = <&gpio1 23 GPIO_ACTIVE_HIGH>;
               label = "ESC1 (CH1)";
           };
           esc2: esc_2 {
               gpios = <&gpio1 25 GPIO_ACTIVE_HIGH>;
               label = "ESC2 (CH2)";
           };
           /* ... more channels ... */
       };
   };

The number of ESC channels is automatically determined from the device tree.

Testing
*******

Unit Tests
==========

Run protocol unit tests:

.. code-block:: console

   west twister -T app/esc_passthrough/tests --platform native_sim

Hardware Integration Tests
==========================

Run Python integration tests (requires hardware):

.. code-block:: console

   cd app/esc_passthrough/tests/pytest
   python -m pytest test_am32_flow.py -v

Or run directly:

.. code-block:: console

   python test_am32_flow.py /dev/ttyACM0

Troubleshooting
***************

ESC Not Detected
================

1. Ensure ESC is powered AFTER the VMU shows "ALL ESC SIGNALS HIGH"
2. Check GPIO connections with ``esc status`` command
3. Verify ESC has AM32 firmware with bootloader support

Write Settings Fails
====================

The "max retries reached" error during writes was caused by a protocol bug
where ``param_count=0`` in responses was interpreted as 256 bytes. This has
been fixed - empty responses now use ``param_count=1`` with a dummy byte.

If you see this error with older firmware, update to the latest version.

Configuration Changes Not Saved
===============================

AM32 EEPROM requires erasing before writing. The application automatically
erases the EEPROM page (0x7C00) on the first write to that region per session.

References
**********

* `AM32 Firmware <https://github.com/am32-firmware/AM32>`_
* `AM32 Bootloader <https://github.com/am32-firmware/AM32-Bootloader>`_
* `AM32 Configurator <https://github.com/am32-firmware/am32-configurator>`_
* `BLHeli Protocol <https://github.com/bitdump/BLHeli>`_
