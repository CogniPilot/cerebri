# SPEC_0004: Tropic Hardware Scope

## Status
ACCEPTED

## Summary
`cerebri2` v1 is locked to the Tropic stack of CRSF, FlexIO DSHOT, ICM45686, and staged M10 GNSS on `mr_vmu_tropic`.

## Specification

**REQUIRED:**
- Board target is `mr_vmu_tropic`.
- RC input uses the Zephyr CRSF driver on `DT_ALIAS(rc)`.
- Motor output uses FlexIO DSHOT only.
- IMU is the onboard `ICM45686`.
- GNSS integration target is the onboard M10 through Zephyr GNSS interfaces.
- Four motor outputs are assumed.

**CURRENT BOARD ASSUMPTIONS:**
- The board DTS exposes the CRSF receiver path as a `tbs,crsf` device even though the node label is legacy `sbus0`.
- Application code consumes the `rc` alias, not the node label.
- DSHOT defaults to `DSHOT600`.
- `DSHOT300` is the first fallback check if ESC signaling is marginal.
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

## Motivation

- Hardware lock reduces ambiguity during bring-up.
- Tropic is the only supported target for first flight.
- A narrow hardware matrix keeps bench debugging tractable.

## References

- `../boards/mr_vmu_tropic.overlay`
- `SPEC_0005_GNSS_STAGING.md`
