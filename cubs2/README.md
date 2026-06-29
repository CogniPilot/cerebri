# CUBS2

`cubs2` is the active multirotor platform folder in this repository.

Start with [spec/README.md](/home/jgoppert/cognipilot/ws/cerebri/cubs2/spec/README.md)
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
- CEP-0002 platform layout under `cubs2/`
- local FlexIO DSHOT driver vendored into this repo
- generated estimator and controller source isolated under `src/generated/`
- CRSF -> generated control -> quad-X mixer -> DSHOT
- `ACRO` and `AUTO_LEVEL` manual flight modes
- GNSS M10 path documented and devicetree-wired through Zephyr GNSS for later use

Build from the workspace root or any child directory inside the same west workspace:

```sh
west build -b mr_vmu_tropic/mimxrt1064 /home/jgoppert/cognipilot/ws/cerebri/cubs2 -d /home/jgoppert/cognipilot/ws/cerebri/cubs2/build
```

## Build Profiles

Source the Zephyr environment before building from this workspace:

```sh
source /home/prady/code/purt/cognipilot/ws/zephyr/zephyr-env.sh
```

Build the software-in-the-loop controller profile. This keeps the internal
CUBS2 bridge RC output protocol:

```sh
west build -b native_sim cubs2 -d build/cubs2-sil -- -DOVERLAY_CONFIG=prj_sil.conf
```

Build the computer-deployment controller profile. This produces a `native_sim`
`zephyr.exe` intended to run on the main computer, consume the current
FlatBuffer controller input path, receive manual control from an external
joystick/manual-control node over csyn, and emit the Arduino PPM encoder serial
packet:

```sh
west build -b native_sim cubs2 -d build/cubs2-deploy -- -DOVERLAY_CONFIG=prj_deploy.conf -DCUBS2_TARGET_NAME=cubs2_deploy
```

Run the computer-deployment binary from that build directory:

```sh
build/cubs2-deploy/zephyr/zephyr.exe
```

## Csyn Native-Sim Bridge

The `native_sim` executable contains the Zephyr-side csyn module and shell. It
does not depend on a scratch checkout under `/tmp`. For Zenoh routing on the
host computer, run the repo-local companion bridge:

```sh
cargo run --manifest-path cubs2/tools/csyn-zephyr-bridge/Cargo.toml -- \
  --connect tcp/127.0.0.1:7447
```

The bridge forwards these Zenoh topics into `zephyr.exe`:

- `synapse/manual_control`
- `synapse/mocap_frame`
- `synapse/sim_input`

It forwards these `zephyr.exe` outputs back onto Zenoh:

- `synapse/flight_snapshot`
- `synapse/motor_output`
- `synapse/control_output`

By default, `zephyr.exe` listens for csyn UDP packets on `127.0.0.1:4250` and
the bridge listens for Zephyr csyn output on `127.0.0.1:4251`. These ports are
controlled by `CUBS2_CSYN_NATIVE_UDP_RX_PORT` and
`CUBS2_CSYN_NATIVE_UDP_TX_PORT`.

To copy the native controls runtime to another computer, package `zephyr.exe`
with the repo-local bridge:

```sh
cubs2/tools/package-native-control.sh build/cubs2-native-sim
scp build/cubs2-native-control-package.tar.gz <host>:
```

On the controls computer:

```sh
tar xf cubs2-native-control-package.tar.gz
cd cubs2-native-control-package
CSYN_CONNECT=tcp/127.0.0.1:7447 ./run-controls.sh
```

Flashing is a separate future path for an onboard microcontroller or a
microcontroller that talks directly to the Arduino PPM encoder. When that
hardware target exists, build the firmware for that board and flash the
resulting build directory. For example:

```sh
west build -b <board> cubs2 -d build/cubs2-onboard -- -DOVERLAY_CONFIG=prj_deploy.conf -DCUBS2_TARGET_NAME=cubs2_onboard
west flash -d build/cubs2-onboard
```

Replace `<board>` with the selected hardware board name, such as
`mr_vmu_tropic/mimxrt1064` if that is the deployment target.

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
west init -l cerebri --mf cubs2/west.yml
west update
west build -b mr_vmu_tropic/mimxrt1064 cerebri/cubs2 -d build/cubs2
```

Important assumptions:
- RC channel map is AETR on CRSF channels 1-4, arm is channel 5, and flight
  mode is channel 6.
- Mixer order is the local default in `src/main.c` and must be verified against the airframe wiring before flight.
- Imported generated estimator and PID source is transitional and should not be
  hand-edited; the intended long-term replacement is local Rumoca-generated
  code.
