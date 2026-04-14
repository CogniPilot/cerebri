# SPEC_0005: GNSS Staging

## Status
DRAFT

## Summary
M10 GNSS support is staged after manual flight bring-up and must stay out of the rate-loop hot path.

## Specification

**REQUIRED:**
- GNSS stays outside the manual-flight inner loop.
- The preferred integration surface is Zephyr GNSS.
- M10 support is developed without adding control-path dependencies.
- If the generic path is insufficient, any dedicated M10 work remains isolated from the rate controller.

**CURRENT DIRECTION:**
- Keep the flight image tolerant of GNSS being absent.
- Avoid noisy placeholder GNSS configurations in the default bench image.
- Bring GNSS online only after CRSF, DSHOT, and IMU behavior are stable.

**PROHIBITED:**
- GNSS polling inside the 1600 Hz control loop.
- GNSS-specific worker threads justified only by convenience.
- Holding first manual flight hostage to GNSS completion.

## Motivation

- GNSS is not required for first manual flight.
- GNSS debugging should not destabilize the core flight stack.
- Staging reduces bring-up risk.

## References

- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
