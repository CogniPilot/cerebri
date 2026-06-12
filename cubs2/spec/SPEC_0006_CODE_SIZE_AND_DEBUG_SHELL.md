# SPEC_0006: Code Size and Debug Shell

## Status
ACCEPTED

## Summary
Source files stay under 2000 lines, and shell-based topic diagnostics are isolated from the flight hot path in a singleton low-priority module.

## Specification

**REQUIRED:**
- Every hand-written `.c` file in this repo stays under 2000 lines.
- When a module grows toward that limit, split by responsibility rather than adding comments or regions to justify size.
- Topic diagnostics live in `src/topic_shell.c`, not in the control-loop implementation file.
- The topic watcher uses exactly one diagnostics thread.
- That diagnostics thread runs at `K_LOWEST_APPLICATION_THREAD_PRIO`.
- `topic echo` and `topic hz` reuse the existing diagnostics thread instead of spawning one per command.
- Runtime shell reads come from stored state, not from adding prints to the control loop.
- Bench motor shell commands live outside `src/main.c`.
- Flight-control debug shell commands live outside `src/main.c`.
- When controller math or device-sampling helpers stop being trivial, move them into focused hot-path modules instead of mixing them with shell or init code in `src/main.c`.
- `src/main.c` should remain an orchestration file that wires IO, mode selection, estimator/controller calls, topic publication, and motor output together.
- In-memory topic state should use small named structs such as `status`, `rate`, and `rc` rather than one flat field bag.
- When data has stable semantic names such as `roll`, `pitch`, and `yaw`, prefer named struct fields over anonymous arrays in published/topic state.
- In hot-path code, prefer one local context object that holds the current-cycle state and publish the latest diagnostics blob from that object instead of maintaining duplicate locals and then copying them field-by-field.
- Shared topic state uses a lockless double-buffered latest-value store with a generation check rather than a mutex-protected shared struct.
- Topic readers retry locally on generation changes instead of blocking the publisher.
- Topic payload layout is governed by `SPEC_0007` and the shared `.fbs` schema files in `modules/lib/synapse_msgs_fbs/fbs/synapse/`.
- Repeated `topic echo` output should use stable fixed-width field formatting so live values are easy to scan.

**PROHIBITED:**
- Per-command topic watcher threads.
- Shell printing from the 800 Hz control loop.
- Folding growing shell/debug code back into `src/main.c`.
- Blocking the control loop on shell/debug state reads.
- Using file size as an excuse to introduce abstraction layers with no ownership benefit.

## Motivation

- The hot path must remain obvious and easy to audit.
- Debug features should not distort flight timing.
- Small source files are easier for humans and AI agents to navigate safely.

## References

- `../src/main.c`
- `../src/control_io.c`
- `../src/fc_shell.c`
- `../src/rate_control.c`
- `../src/topic_shell.c`
- `SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
