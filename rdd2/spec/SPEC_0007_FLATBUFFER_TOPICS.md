# SPEC_0007: FlatBuffer Topic Format

## Status
ACCEPTED

## Summary
Topic payloads are published as fixed-size FlatBuffer-compatible blobs, while the control loop keeps native local state in `ctx`.

## Specification

**REQUIRED:**
- Topic schemas live in `modules/lib/synapse_msgs_fbs/fbs/synapse/` as `.fbs` files.
- Generated FlatBuffer headers live under `${CMAKE_BINARY_DIR}/generated/flatbuffers`.
- Generated `.bfbs` schemas for topic/log reflection also live under `${CMAKE_BINARY_DIR}/generated/flatbuffers` when `flatcc` is available.
- Shared topic storage uses FlatBuffer-compatible binary blobs, not shared native structs.
- When a topic schema defines a fixed struct, firmware code must use the generated flatcc struct type instead of a handwritten mirror.
- The control loop may keep native local state, but the published diagnostics blob must be encoded before it enters the shared topic store.
- FlatBuffer topic publication must remain heap-free and bounded.
- Fixed-size topic tables must be encoded and decoded through generated FlatCC APIs, not handwritten offset logic.
- If topic encoding runs in the rate-loop hot path, it must use a bounded preallocated FlatCC builder path rather than heap-backed dynamic builder state.
- V1 topic schemas use fixed-size scalar-only fields and fixed-size structs.
- Topic readers decode from the shared FlatBuffer blob after copying it locally.
- The published flight-state topic should carry the current flight mode, estimated attitude, desired attitude, desired rates, and commanded rates needed for bench and SITL debugging.
- The published flight-state topic should also carry the measured hot-path latency from the IMU interrupt timestamp to the DSHOT trigger timestamp in microseconds.

**PROHIBITED:**
- Heap allocation or dynamic builders in the rate-loop hot path.
- Writing generated FlatBuffer code into the source tree.
- Hand-packed FlatBuffer table encoders or decoders for schema-defined topic payloads.
- Handwritten field offset maps for schema-defined FlatBuffer tables.
- Strings, variable-length vectors, or reflection-driven decoding in v1 topic blobs.
- Sharing hot-path native structs directly with diagnostics readers.

## Motivation

- FlatBuffer schemas give AI and humans a stable topic contract.
- Fixed-size encodings keep the latency cost bounded and easy to reason about.
- Native local state in `ctx` keeps the control loop simple and fast.

## References

- `../../modules/lib/synapse_msgs_fbs/fbs/synapse/synapse_topics.fbs`
- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md`
