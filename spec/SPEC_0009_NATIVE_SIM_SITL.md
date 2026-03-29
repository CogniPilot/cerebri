# SPEC_0009: Native Sim SITL Transport

## Status
ACCEPTED

## Summary
`native_sim` runs the same 1 kHz rate-mode loop as flight firmware, but simulator IO terminates in native-sim RC, IMU, and DSHOT devices backed by one low-priority UDP coordinator and FlatBuffer schemas in `synapse_msgs_fbs`.

## Specification

**REQUIRED:**
- The `native_sim` target is for SITL and debug only.
- The controller still runs in the main 1 kHz application thread on `native_sim`.
- Host simulator IO must stay out of the hot path and run in a lower-priority thread than the main control loop.
- `src/main.c` must remain free of `native_sim`-specific control-path branches.
- The simulator boundary must terminate at board-selected `rc`, `imu0`, and `motors` devices, not at ad hoc app-level IO hooks.
- Inbound simulator packets use `fbs/cerebri2/cerebri2_sil.fbs` `SimInput`.
- Outbound simulator packets reuse `cerebri2_topics.fbs` `FlightSnapshot` and `MotorOutput`.
- SITL message schemas live in `modules/lib/synapse_msgs_fbs/fbs/cerebri2/`.
- The SITL UDP coordinator must stage the latest inbound `SimInput` as a FlatBuffer blob instead of queueing per-sample work into the controller.
- Shared SITL state between the UDP coordinator and fake drivers must remain FlatBuffer blobs, not shared native structs.
- Driver-local fetch code may decode transient native locals from the staged FlatBuffer blob.
- SITL board-specific configuration lives in `boards/native_sim.conf`.
- SITL device selection lives in `boards/native_sim.overlay`.

**PROHIBITED:**
- Blocking socket IO in the 1 kHz control loop.
- A simulation-only control loop separate from `src/main.c`.
- SITL message definitions that bypass `modules/lib/synapse_msgs_fbs`.
- Per-packet heap allocation in the hot path.
- Publishing simulator UDP traffic directly from `src/main.c`.

## Motivation

- SITL is valuable for fast iteration, but it should exercise the same controller code as flight firmware.
- Driver-backed SITL keeps the simulator boundary explicit without polluting the controller.
- FlatBuffer-backed shared state keeps simulator transport, fake drivers, and tooling on one contract.
- Reusing the shared FlatBuffer schemas keeps firmware, tooling, and ROS mirrors aligned.

## References

- `../boards/native_sim.conf`
- `../boards/native_sim.overlay`
- `../src/main.c`
- `../src/rc_input.c`
- `../src/sitl_flatbuffer.c`
- `../src/sitl_udp_coordinator.h`
- `../drivers/sitl_udp_coordinator.c`
- `../drivers/sitl_rc.c`
- `../drivers/sitl_imu.c`
- `../drivers/sitl_dshot.c`
- `../../modules/lib/synapse_msgs_fbs/fbs/cerebri2/cerebri2_sil.fbs`
- `../../modules/lib/synapse_msgs_fbs/fbs/cerebri2/cerebri2_topics.fbs`
- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
- `SPEC_0007_FLATBUFFER_TOPICS.md`
