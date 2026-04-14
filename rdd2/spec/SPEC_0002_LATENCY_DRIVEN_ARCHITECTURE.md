# SPEC_0002: Latency-Driven Architecture

## Status
ACCEPTED

## Summary
Minimal latency is a primary system driver, so the flight stack keeps one application hot-path thread, paces the body-rate loop from the IMU data-ready interrupt at 1600 Hz, and runs body-rate control, mixing, and motor output in that same thread with no extra handoff. Slower estimator, auto-level, and diagnostics work may run at integer divisors of that loop in the same thread.

## Specification

**REQUIRED:**
- The application hot path uses one thread only: the main control loop in `src/main.c`.
- On `mr_vmu_tropic`, the main body-rate loop is paced by the `ICM45686` data-ready interrupt at 1600 Hz.
- Every body-rate loop iteration consumes the latest decoded IMU sample, runs the body-rate PID, mixes outputs, and triggers DSHOT in that one thread.
- Estimator prediction and outer attitude control may run at lower integer-divisor rates in the same thread when that reduces hot-path cost without adding handoff latency.
- In manual-flight operation, attitude prediction must stay decimated relative to the 1600 Hz body-rate loop and must not run every control iteration unless a measured need justifies it.
- Once IMU pacing is active on `mr_vmu_tropic`, the hot path does not add a second fixed-period sleep on top of that pacing source.
- RC handoff into the app is a bounded latest-sample update, not a queue.
- Hot-path publication to diagnostics uses one lockless publish step into a double-buffered latest-value store.
- Diagnostics readers consume the latest published data without blocking the control loop.
- Diagnostics publication may be decimated relative to the body-rate loop and must not gate motor output.
- Hot-path math uses `float`, not `double`.
- No heap allocation occurs after boot.
- Hot-path synchronization points stay explicit and few.
- If attitude correction exists, it must stay out of the 1600 Hz body-rate hot path and must not gate motor output.

**ALLOWED:**
- Existing Zephyr driver threads that already belong to subsystems such as CRSF.
- One low-priority diagnostics thread outside the flight hot path when required by `SPEC_0006`.

**PROHIBITED:**
- Additional application threads in the flight hot path.
- Queues between RC state, estimator, controller, mixer, and motor output.
- Mutexes, semaphores, or workqueues in the hot-path publish path.
- Synchronous IMU bus reads from the flight control loop on `mr_vmu_tropic`.
- Periodic shell or log output from the 1600 Hz body-rate loop.
- Kalman or measurement-correction steps that block the IMU-paced control iteration.
- Adding latency-oriented abstractions without measured justification on `mr_vmu_tropic`.

## Motivation

- Threading and queueing cost latency and jitter.
- The repo is meant to optimize manual-flight responsiveness first.
- Measurements, not abstraction preference, decide future complexity.

## References

- `SPEC_0003_RATE_MODE_CONTROL_SCOPE.md`
- `SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md`
