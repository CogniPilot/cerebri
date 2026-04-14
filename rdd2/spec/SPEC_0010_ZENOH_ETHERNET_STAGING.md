# SPEC_0010: Zenoh Ethernet Staging

## Status
DRAFT

## Summary
Zenoh over Ethernet is staged as a low-priority UDP subscriber on Tropic and must stay out of the flight hot path until a concrete offboard-message contract is defined.

## Specification

**REQUIRED:**
- Zenoh-over-Ethernet uses the standard Zephyr IPv4/UDP networking stack on `mr_vmu_tropic`.
- The Zenoh integration uses `zenoh-pico`, not a custom wire parser.
- Session bring-up and reconnect run in a dedicated low-priority thread.
- Received data is retained as a bounded latest-sample snapshot for inspection and later decoding.
- Shell inspection reads stored state rather than polling the network path directly.
- The shell exposes a small Zenoh operator CLI for bring-up: configuration, status, latest sample inspection, sample clear, and debug publish.

**CURRENT DIRECTION:**
- Default bench configuration uses static IPv4 settings to simplify bring-up.
- The initial integration stores raw Zenoh payload bytes plus metadata until the mocap message contract is pinned down.
- The first target is reliable Ethernet and subscription bring-up, not control-loop coupling.
- Debug shell publishes are allowed for bench validation, but they must stay outside the flight hot path and may use an ephemeral session.
- Board-level Zephyr bring-up shells such as `net ping`, `net stats`, and `mdio` are allowed for Ethernet diagnostics, but subsystem-specific transport shells should continue to read stored state rather than driving the live path directly.

**PROHIBITED:**
- Blocking the 1600 Hz control loop on Ethernet or Zenoh traffic.
- Decoding Zenoh payloads inside the flight hot path by default.
- Unbounded payload buffering or per-sample heap ownership in shell/debug code.

## Motivation

- Mocap and offboard position inputs are useful, but bring-up should not destabilize manual flight.
- Zenoh transport debugging is easier when network and subscription state can be inspected independently from controller behavior.
- A bounded latest-sample store is enough for initial integration and shell diagnostics.

## References

- `../../../modules/lib/zenoh-pico/src/system/zephyr/zenoh_pico_shell_backend.c`
- `../../../modules/lib/zenoh-pico/src/system/zephyr/zenoh_pico_shell.c`
- `../../../modules/lib/zenoh-pico/zephyr/Kconfig.zenoh`
- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
