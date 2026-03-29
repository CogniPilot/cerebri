# cerebri

`cerebri` is a clean-slate Zephyr multirotor codebase for `mr_vmu_tropic`.

Start with [spec/README.md](/home/jgoppert/cognipilot/ws/cerebri/spec/README.md) for project rules and [AGENTS.md](/home/jgoppert/cognipilot/ws/cerebri/AGENTS.md) for contributor/agent workflow.

V1 goals:
- `mr_vmu_tropic` only
- CRSF input only
- DSHOT output only
- ICM45686 IMU only
- one application hot-path thread
- no dependency on the legacy `cerebri` module
- no `double` in the control path

Current implementation scope:
- fresh repo layout with `spec/` and `dev/`
- local FlexIO DSHOT driver vendored into this repo
- minimal manual rate-mode controller
- CRSF -> rate PID -> quad-X mixer -> DSHOT
- GNSS M10 path documented and devicetree-wired through Zephyr GNSS for later use

Build from the workspace root or any child directory inside the same west workspace:

```sh
west build -b mr_vmu_tropic/mimxrt1064 /home/jgoppert/cognipilot/ws/cerebri -d /home/jgoppert/cognipilot/ws/cerebri/build
```

If `flatcc` is installed, CMake stages generated FlatBuffer headers and a copy of the
active schemas from `modules/lib/synapse_msgs_fbs/fbs/cerebri` under
`${CMAKE_BINARY_DIR}/generated/flatbuffers`. Generated files are not kept in the source
tree.

The same `flatcc` tool also generates `.bfbs` binary schemas for
`cerebri_topics.fbs` and `cerebri_log.fbs` from that module in the same build-tree
directory for self-describing SD-card log streams.

To bootstrap a fresh minimal workspace from this repo's manifest, check out this repo at
`<workspace>/cerebri` and initialize west from the workspace root:

```sh
mkdir -p /tmp/cerebri-ws
git clone <repo-url> /tmp/cerebri-ws/cerebri
cd /tmp/cerebri-ws
west init -l cerebri
west update
west build -b mr_vmu_tropic/mimxrt1064 cerebri -d build/cerebri
```

Important assumptions:
- RC channel map is AETR on CRSF channels 1-4 and arm is channel 5.
- Mixer order is the local default in `src/main.c` and must be verified against the airframe wiring before flight.
- The current flight stack is rate mode first. Outer attitude hold and a quaternion Kalman attitude estimator remain planned work in `dev/roadmap.md`.
