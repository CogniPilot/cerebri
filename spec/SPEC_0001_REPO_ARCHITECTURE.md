# SPEC_0001: Repository Architecture

## Status
ACCEPTED

## Summary
`cerebri` is a single-airframe Zephyr app whose flight hot path stays simple, local, and independent from the legacy `cerebri` module.

## Specification

**REQUIRED:**
- The root Zephyr application lives at the repository top level.
- `src/main.c` owns the v1 flight hot path.
- Local app build wiring lives in `src/CMakeLists.txt`, not in one growing root source list.
- Local subsystem build/config wiring lives under `subsys/` with local `CMakeLists.txt` and `Kconfig` files.
- Local driver build/config wiring lives under `drivers/` with family-local `CMakeLists.txt` and `Kconfig` files where needed.
- Debug and shell helpers live outside the hot-path module when they grow beyond trivial size.
- The Tropic FlexIO DSHOT driver family remains vendored locally under `drivers/nxp_flexio_dshot/`.
- `spec/` is the source of truth for project rules.
- `dev/` holds roadmaps and bring-up execution material.
- The repo targets one board in v1: `mr_vmu_tropic`.

**PROHIBITED:**
- Dependency on the legacy `cerebri` module.
- Generic middleware buses or message brokers in the flight stack.
- Multi-board abstraction layers for v1.
- Navigation, autonomy, or mission code in the manual-flight hot path.

## Motivation

- The codebase should be easy to audit.
- AI agents need obvious ownership boundaries.
- Tropic bring-up should not be blocked by architecture speculation.

## References

- `../src/CMakeLists.txt`
- `../subsys/CMakeLists.txt`
- `../subsys/Kconfig`
- `../drivers/CMakeLists.txt`
- `../drivers/Kconfig`
- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
