# SPEC_0003: Rate-Mode Control Scope

## Status
ACCEPTED

## Summary
The first flight-capable stack is manual multirotor rate mode with CRSF stick input, body-rate PID, quad-X mixing, and hard disarm/failsafe behavior.

## Specification

**REQUIRED:**
- Manual flight only in v1.
- Inner body-rate PID is the active controller.
- Mixer shape is quad-X.
- Control-path state is fixed-size and explicit.
- Control code consumes body rates in `FRD` body axes.
- Roll and pitch commands come from CRSF stick inputs.
- Yaw uses its own rate PID path.
- Motor order at the control layer is:
  - `m0 = front-right`
  - `m1 = rear-right`
  - `m2 = rear-left`
  - `m3 = front-left`
- Quad-X roll/pitch mix signs follow `FRD` right-hand-rule torques:
  - positive roll increases left motors and decreases right motors
  - positive pitch increases front motors and decreases rear motors

**PROHIBITED:**
- `double` in the control path.
- Position control in the hot path.
- Velocity control in the hot path.
- Generic control-allocation frameworks in v1.
- GPS or navigation fusion in the rate-loop hot path.

## Motivation

- First hover should happen with the smallest viable flight stack.
- Body-rate control is the shortest path to flight validation.
- More layers belong after basic flight behavior is proven.

## References

- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
