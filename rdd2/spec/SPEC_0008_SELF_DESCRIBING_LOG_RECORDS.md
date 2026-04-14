# SPEC_0008: Self-Describing Log Records

## Status
ACCEPTED

## Summary
SD-card logging uses a dedicated FlatBuffer log-envelope schema plus generated binary schemas so recorded data can be decoded later without private source-tree knowledge.

## Specification

**REQUIRED:**
- Log records use `modules/lib/synapse_msgs_fbs/fbs/synapse/synapse_log.fbs`, not ad-hoc raw byte dumps.
- Logged topic payloads are wrapped in a typed FlatBuffer union record.
- The log format supports a `SchemaRecord` that carries the generated `.bfbs` schema bytes.
- Build outputs place generated `.bfbs` files under `${CMAKE_BINARY_DIR}/generated/flatbuffers`.
- The logger remains outside the flight hot path.

**PROHIBITED:**
- Writing anonymous topic bytes to storage with no typed envelope.
- Doing reflection parsing or schema serialization in the rate-loop hot path.

## Motivation

- A log file should still be decodable after the source tree changes.
- A typed log envelope avoids guessing payload type from filename or offset.
- Schema records let offline tools recover structure directly from the log stream.

## References

- `../../modules/lib/synapse_msgs_fbs/fbs/synapse/synapse_log.fbs`
- `../../modules/lib/synapse_msgs_fbs/fbs/synapse/synapse_topics.fbs`
- `SPEC_0007_FLATBUFFER_TOPICS.md`
