# RDD2

`rdd2` is the active multirotor platform folder in this repository.

Start with [spec/README.md](/home/jgoppert/cognipilot/ws/cerebri/rdd2/spec/README.md)
for platform rules.

V1 goals:
- `mr_vmu_tropic` only
- CRSF input only
- DSHOT output only
- ICM45686 IMU only
- one application hot-path thread
- no dependency on the legacy `cerebri` module
- no `double` in the control path

Current implementation scope:
- CEP-0002 platform layout under `rdd2/`
- local FlexIO DSHOT driver vendored into this repo
- generated estimator and controller source isolated under `src/generated/`
- CRSF -> generated control -> quad-X mixer -> DSHOT
- `ACRO` and `AUTO_LEVEL` manual flight modes
- GNSS M10 path documented and devicetree-wired through Zephyr GNSS for later use

Build from the workspace root or any child directory inside the same west workspace:

```sh
west build -b mr_vmu_tropic/mimxrt1064 /home/jgoppert/cognipilot/ws/cerebri/rdd2 -d /home/jgoppert/cognipilot/ws/cerebri/rdd2/build
```

If `flatcc` is installed, CMake stages generated FlatBuffer headers and a copy of the
active schemas from `modules/lib/synapse_msgs_fbs/fbs/synapse` under
`${CMAKE_BINARY_DIR}/generated/flatbuffers`. Generated files are not kept in the source
tree.

The same `flatcc` tool also generates `.bfbs` binary schemas for
`synapse_topics.fbs` and `synapse_log.fbs` from that module in the same build-tree
directory for self-describing SD-card log streams.

To bootstrap a fresh minimal workspace from this repo's manifest, check out
this repo at `<workspace>/cerebri` and initialize west from the workspace root
with the platform manifest file:

```sh
mkdir -p /tmp/cerebri-ws
git clone <repo-url> /tmp/cerebri-ws/cerebri
cd /tmp/cerebri-ws
west init -l cerebri --mf rdd2/west.yml
west update
west build -b mr_vmu_tropic/mimxrt1064 cerebri/rdd2 -d build/rdd2
```

Important assumptions:
- RC channel map is AETR on CRSF channels 1-4, arm is channel 5, and flight
  mode is channel 6.
- Mixer order is the local default in `src/main.c` and must be verified against the airframe wiring before flight.
- Imported generated estimator and PID source is transitional and should not be
  hand-edited; the intended long-term replacement is local Rumoca-generated
  code.
