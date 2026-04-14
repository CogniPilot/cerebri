# SPEC_0004: Tropic Hardware Scope

## Status
ACCEPTED

## Summary
`RDD2` v1 flight hardware is locked to the Tropic stack of CRSF, FlexIO DSHOT, ICM45686, and staged M10 GNSS on `mr_vmu_tropic`, while `native_sim` is allowed only for non-flight SITL.

## Specification

**REQUIRED:**
- `mr_vmu_tropic` is the only supported flight-hardware target.
- `native_sim` is allowed only for SITL and debug builds.
- RC input uses the Zephyr CRSF driver on `DT_ALIAS(rc)`.
- Motor output uses FlexIO DSHOT only.
- IMU is the onboard `ICM45686`.
- GNSS integration target is the onboard M10 through Zephyr GNSS interfaces.
- Four motor outputs are assumed.

**CURRENT BOARD ASSUMPTIONS:**
- The board DTS exposes the CRSF receiver path as a `tbs,crsf` device even though the node label is legacy `sbus0`.
- Application code consumes the `rc`, `imu0`, and `motors` aliases, not board-specific node labels.
- DSHOT defaults to `DSHOT600`.
- `DSHOT300` is the first fallback check if ESC signaling is marginal.
- The `ICM45686` gyro and accel run at `1600 Hz` ODR for the flight loop.
- The `ICM45686` data-ready interrupt advances one `1600 Hz` control iteration.
- The onboard Ethernet path, when enabled, uses the `TJA1103` PHY with an external RMII reference clock.
- The Tropic `TJA1103` reset line is wired to `GPIO_B0_14` / `GPIO2_IO14` and must be driven high before PHY initialization.
- IMU data is remapped into `FRD` body axes before control use:
  - body roll / `x` = sensor gyro `y`
  - body pitch / `y` = sensor gyro `x`
  - body yaw / `z` = `-sensor gyro z`
- Accel uses the same axis remap as gyro.

**PROHIBITED:**
- PWM support.
- Alternate RC protocols in v1.
- Alternate motor-protocol abstractions in v1.
- Adding legacy `cerebri` hardware shims.
- Treating `native_sim` as a supported flight-hardware target.

## Motivation

- Hardware lock reduces ambiguity during bring-up.
- Tropic is the only supported target for first flight.
- `native_sim` is useful for controller and tooling iteration, but it must not loosen the flight-hardware contract.
- A narrow hardware matrix keeps bench debugging tractable.

## References

- `../boards/mr_vmu_tropic.overlay`
- `SPEC_0009_NATIVE_SIM_SITL.md`
- `SPEC_0005_GNSS_STAGING.md`
