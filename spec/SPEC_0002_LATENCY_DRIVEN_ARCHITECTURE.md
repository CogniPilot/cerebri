# SPEC_0002: Latency-Driven Architecture

## Status
ACCEPTED

## Summary
Minimal latency is a primary system driver, so the flight stack keeps one application hot-path thread and removes unnecessary synchronization from RC input to motor output.

## Specification

**REQUIRED:**
- The application hot path uses one thread only: the main control loop in `src/main.c`.
- The control loop owns IMU polling, PID, mixing, and DSHOT trigger.
- RC handoff into the app is a bounded snapshot update, not a queue.
- Hot-path publication to diagnostics uses one lockless snapshot publish step into a double-buffered store.
- Diagnostics readers consume published snapshots without blocking the control loop.
- Hot-path math uses `float`, not `double`.
- No heap allocation occurs after boot.
- Hot-path synchronization points stay explicit and few.

**ALLOWED:**
- Existing Zephyr driver threads that already belong to subsystems such as CRSF.
- One low-priority diagnostics thread outside the flight hot path when required by `SPEC_0006`.

**PROHIBITED:**
- Additional application threads in the flight hot path.
- Queues between RC state, estimator, controller, mixer, and motor output.
- Mutexes, semaphores, or workqueues in the hot-path snapshot publish path.
- Periodic shell or log output from the 1 kHz control loop.
- Adding latency-oriented abstractions without measured justification on `mr_vmu_tropic`.

## Motivation

- Threading and queueing cost latency and jitter.
- The repo is meant to optimize manual-flight responsiveness first.
- Measurements, not abstraction preference, decide future complexity.

## References

- `SPEC_0003_RATE_MODE_CONTROL_SCOPE.md`
- `SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md`
