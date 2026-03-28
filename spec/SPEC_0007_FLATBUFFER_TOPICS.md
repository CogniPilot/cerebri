# SPEC_0007: FlatBuffer Topic Format

## Status
ACCEPTED

## Summary
Topic payloads are published as fixed-size FlatBuffer-compatible blobs, while the control loop keeps native local state in `ctx`.

## Specification

**REQUIRED:**
- Topic schemas live in `modules/lib/synapse_msgs_fbs/fbs/cerebri2/` as `.fbs` files.
- Generated FlatBuffer headers live under `${CMAKE_BINARY_DIR}/generated/flatbuffers`.
- Generated `.bfbs` schemas for topic/log reflection also live under `${CMAKE_BINARY_DIR}/generated/flatbuffers` when `flatcc` is available.
- Shared topic storage uses FlatBuffer-compatible binary blobs, not shared native structs.
- The control loop may keep native local state, but the published diagnostics snapshot must be encoded before it enters the shared topic store.
- FlatBuffer topic publication must remain heap-free and bounded.
- V1 topic schemas use fixed-size scalar-only fields and fixed-size structs.
- Topic readers decode from the shared FlatBuffer blob after snapshotting it locally.

**PROHIBITED:**
- Heap allocation or dynamic builders in the 1 kHz control loop.
- Writing generated FlatBuffer code into the source tree.
- Strings, variable-length vectors, or reflection-driven decoding in v1 topic blobs.
- Sharing hot-path native structs directly with diagnostics readers.

## Motivation

- FlatBuffer schemas give AI and humans a stable topic contract.
- Fixed-size encodings keep the latency cost bounded and easy to reason about.
- Native local state in `ctx` keeps the control loop simple and fast.

## References

- `../../modules/lib/synapse_msgs_fbs/fbs/cerebri2/cerebri2_topics.fbs`
- `../src/topic_flatbuffer.c`
- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md`
