# SPEC_0003: Manual Flight Control Scope

## Status
ACCEPTED

## Summary
The first flight-capable stack is manual multirotor flight with CRSF stick input, a handwritten fixed-step body-rate PID, quad-X mixing, a lightweight handwritten attitude predictor, and two pilot-selectable modes: `ACRO` and `AUTO_LEVEL`.

## Specification

- Manual flight only in v1.
- `CH5` is the flight-mode switch.
- `CH5 < 1500` selects `ACRO`.
- `CH5 >= 1500` selects `AUTO_LEVEL`.
- Inner body-rate PID is the active controller in both modes.
- `ACRO` commands body-rate setpoints directly from RC sticks.
- `AUTO_LEVEL` commands roll and pitch attitude through an outer attitude controller and keeps yaw as a rate command.
- Attitude prediction runs in the hot path from gyro and accelerometer data using handwritten fixed-step local code.
- Attitude correction is not required in the hot path for v1 auto-level.
- Rate and attitude PID control in the flight stack use handwritten fixed-step discrete controllers, not generated multi-stage integrators.
- Mixer shape is quad-X.
- Control-path state is fixed-size and explicit.
- Control code consumes body rates in `FLU` body axes and world references in `ENU`.
- Roll and pitch commands come from CRSF stick inputs.
- Yaw uses its own rate PID path in both modes.
- Motor order at the control layer is:
  - `m0 = front-right`
  - `m1 = rear-right`
  - `m2 = rear-left`
  - `m3 = front-left`
- Quad-X roll/pitch mix signs follow `FLU` right-hand-rule torques:
  - positive roll increases left motors and decreases right motors
  - positive pitch increases rear motors and decreases front motors

**PROHIBITED:**
- `double` in the control path.
- Position control in the hot path.
- Velocity control in the hot path.
- Generic control-allocation frameworks in v1.
- GPS or navigation fusion in the rate-loop hot path.
- Attitude correction or other estimator update steps that block the 1600 Hz body-rate loop.

## Motivation

- First hover should happen with the smallest viable flight stack.
- `AUTO_LEVEL` adds a safer bench and early-flight mode without bringing in full navigation.
- Prediction-only attitude state keeps the hot path fast while leaving room for later off-path correction.

## References

- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
