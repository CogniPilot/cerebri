# NXP FlexIO DShot Driver

Zephyr driver for generating DShot motor control signals using the NXP FlexIO
peripheral.  The driver targets the `cognipilot,flexio-dshot` devicetree
compatible and is built when `CONFIG_CEREBRI_DSHOT=y` (default when a FlexIO
node is present).  A lightweight SITL stub (`cognipilot,sitl-dshot`) is compiled
instead when `CONFIG_CEREBRI_SITL=y` so native\_sim builds can exercise the
motor-output path without hardware.

## Supported hardware

| SoC | FlexIO clock setup | Notes |
|---|---|---|
| MIMXRT1064 | PLL3 PFD2, 108 MHz | MR-VMU-RT1064 boards |
| MIMXRT1176 | SysPLL2 PFD3, 108 MHz | V6X-RT class boards |
| MIMX9596 (M7) | SYSPLL1\_PFD1\_DIV2, 133 MHz | i.MX95 targets via SCMI |

## Protocol overview

DShot is a digital protocol for communicating with brushless ESCs.  Each frame
is a 16-bit word:

| Bits | Field |
|---|---|
| 15-5 | Throttle (0 = disarmed, 1-47 reserved/commands, 48-2047 throttle) |
| 4    | Telemetry request |
| 3-0  | XOR nibble checksum |

The driver bit-bangs DShot via FlexIO shifters and timers in Dual 8-bit Baud/Bit
mode.  Each bit of the 16-bit packet is expanded into a 3-clock-cycle PWM symbol
so the ESC can distinguish a `1` (two high, one low) from a `0` (one high, two
low).  The first 24 bits are loaded into the shifter buffer; the remaining 24
bits are loaded via a shifter-status interrupt (two 32-bit writes total for 48
FlexIO clocks).

### Supported speeds

The `speed` devicetree property sets the bitrate in kbit/s.  Common values:

- **600** -- DShot600 (default)
- **300** -- DShot300 (fallback)

## Bidirectional DShot (BDShot)

When `bidirectional-dshot` is set on a channel node, the driver inverts the
output polarity so the ESC knows to reply with telemetry on the same wire.
After the 16-bit transmit frame completes, the ISR reconfigures the FlexIO
shifter and timer from transmit to receive mode, captures a 20-bit GCR-encoded
response, and decodes it into electrical RPM (eRPM).

The driver decodes the GCR response into per-channel eRPM values internally.
The application layer does not currently consume eRPM data; integration into
the control loop or a telemetry topic is future work.

### BDShot baudrate training

ESC hardware varies -- AM32 firmware in particular lacks clock compensation, so
the optimal receive baudrate differs from unit to unit.  The driver performs
automatic per-channel baudrate training at startup:

1. The timer-compare offset sweeps from -16 to +15 around the nominal value.
2. At each offset, 200 frames are sent.  The raw response is compared against
   the known zero-throttle pattern (`0x52951`).
3. An offset is marked valid when >= 198 of 200 responses match.
4. After the sweep, the midpoint of the valid offset range is selected.
5. If no offset passes, the sweep restarts automatically.

Training runs transparently before normal eRPM decoding begins.  A log message
is emitted when training completes for each channel.  During training the
channel does not report eRPM values.

## Devicetree configuration

```dts
&flexio1 {
    status = "okay";

    dshot: dshot {
        compatible = "cognipilot,flexio-dshot";
        pinctrl-0 = <&pinmux_dshot>;
        pinctrl-names = "default";
        status = "okay";
        speed = <600>;

        ch1 {
            pin-id = <6>;
            bidirectional-dshot;
        };
        ch2 {
            pin-id = <7>;
            bidirectional-dshot;
        };
        /* ... */
    };
};
```

### Properties

| Property | Type | Description |
|---|---|---|
| `speed` | int | Bitrate in kbit/s (default 600) |

### Per-channel properties

| Property | Type | Description |
|---|---|---|
| `pin-id` | int | FlexIO data pin index (FXIO\_D\<n\>) |
| `bidirectional-dshot` | boolean | Enable BDShot telemetry on this channel |

## Driver API

The driver extends the Zephyr Sensor API with three additional entry points
defined in `nxp_flexio_dshot.h`:

```c
/* Set throttle for a channel (0 = disarmed, 48-2047 = throttle). */
void nxp_flexio_dshot_data_set(const struct device *dev,
                               unsigned channel,
                               uint16_t throttle,
                               bool telemetry);

/* Trigger transmission on all channels simultaneously. */
void nxp_flexio_dshot_trigger(const struct device *dev);

/* Return the number of configured channels. */
uint8_t nxp_flexio_dshot_channel_count(const struct device *dev);
```

Typical usage from the motor-output layer:

```c
for (int i = 0; i < 4; i++) {
    nxp_flexio_dshot_data_set(dev, i, dshot_value[i], false);
}
nxp_flexio_dshot_trigger(dev);
```

## File layout

| File | Purpose |
|---|---|
| `nxp_flexio_dshot.c` | Core driver: FlexIO configuration, ISR, BDShot training and decode |
| `nxp_flexio_dshot_api.c` | Thin public API wrappers |
| `sitl_dshot.c` | No-op stub for native\_sim builds |
| `Kconfig` | `CONFIG_CEREBRI_DSHOT` option |
| `CMakeLists.txt` | Conditional compilation |

## Current state and known limitations

- BDShot baudrate training has been implemented but not yet validated on
  hardware with AM32 ESCs.  It has been tested with BLHeli32 ESCs on MR-VMU
  Tropic boards.
- The ISR-driven transmit/receive path works for up to 4 channels on a single
  FlexIO instance.  More channels would require a second FlexIO or a different
  scheduling strategy.
- DShot commands (beep, spin direction, save settings) are defined in the header
  but the application layer does not yet expose a shell or topic interface for
  sending them on demand.
- SITL stub returns zero eRPM for all channels; it does not simulate ESC
  response dynamics.
