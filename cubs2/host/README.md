# cubs2_host

Native (Linux) fixed-wing outer-loop controller — a drop-in replacement for the
ROS/Python `cub_tecs_ros_xtrack.py` node.

```
synapse_qualisys_bridge ──(Zenoh: MocapFrame flatbuffer)──▶ cubs2_host
cubs2_host ──(57600 serial: 0xFF-framed RC)──▶ ppm_bridge Arduino ──▶ PPM ──▶ Tx
```

It runs the **same** generated controller as the embedded `cubs2` firmware
(`src/generated_fixed_wing/CubControl_FixedWingOuterLoop`) with the **same**
parameter set, at 100 Hz, with no ROS and no Zephyr.

- **Input:** subscribes (zenoh-pico) to `synapse/mocap/frame`, decodes the
  `MocapFrame` flatbuffer published by CogniPilot/synapse_qualisys_bridge, and
  feeds rigid-body 0 (position + quaternion) into the controller.
- **Output:** maps the controller's AETR + stabilizer to a 14-byte PPM frame
  (`0xFF 0xFF | 5×uint16 LE | uint16 LE checksum`) at 57600 8N1 — exactly what
  the `wsribunma/ppm_bridge` Arduino expects.
- Before the first mocap frame (or if the session can't open) it emits a
  failsafe frame: throttle low, sticks centered.

## Build

```sh
./build.sh        # builds flatcc + zenoh-pico into third_party/ on first run
```

Output: `build/cubs2_host`. Requires a C compiler, cmake, ninja/make, git.

## Run

```sh
# defaults: device /dev/ttyACM0, key synapse/mocap/frame, zenoh peer mode
./build/cubs2_host

# explicit serial device as argv[1]
./build/cubs2_host /dev/ttyACM0
```

Environment overrides:

| Variable               | Default               | Meaning                                  |
|------------------------|-----------------------|------------------------------------------|
| `CUBS2_PPM_DEVICE`     | `/dev/ttyACM0`        | PPM bridge serial port (or argv[1])      |
| `CUBS2_MOCAP_KEY`      | `synapse/mocap/frame` | Zenoh key to subscribe to                |
| `CUBS2_ZENOH_MODE`     | `peer`                | `peer` or `client`                       |
| `CUBS2_ZENOH_CONNECT`  | (unset)               | endpoint, e.g. `tcp/127.0.0.1:7447`      |

If your network has no multicast, run a zenoh router and use
`CUBS2_ZENOH_MODE=client CUBS2_ZENOH_CONNECT=tcp/<router-ip>:7447` (point the
qualisys bridge at the same router).

A non-tty `CUBS2_PPM_DEVICE` (file, pipe, `/dev/null`) is allowed for testing —
raw frames are written without serial line-discipline setup.

## Testing the mocap path (no hardware)

`test/zenoh_loopback.sh` runs a synthetic mocap publisher
(`test/mocap_pub.c`, byte-identical to the qualisys bridge) and `cubs2_host`
together, then checks that mocap is received and RC outputs leave failsafe:

```sh
test/zenoh_loopback.sh
```

By default it uses a router-free TCP-loopback Zenoh transport (publisher
listens, controller connects) so it works without multicast or a router. To
test against a real router instead:

```sh
docker run --rm --net host eclipse/zenoh:latest &        # or any zenoh router
CUBS2_ZENOH_MODE=client CUBS2_ZENOH_CONNECT=tcp/127.0.0.1:7447 test/zenoh_loopback.sh
```

This proves transport + decode + controller ingestion. It does **not** validate
flight dynamics — the synthetic pose is not physically consistent, so the
controller's state estimator (finite differences on position) produces
unrealistic velocities and the flight-mode/throttle outputs are not meaningful.

Note the takeoff→airborne transition is **edge-triggered** (`z` crossing
`takeoffAltitude` from below). Start `cubs2_host` while the aircraft is on the
ground so it observes the crossing.

## Verified

- Builds to a standalone executable; controller + flatbuffer decode + PPM
  output + zenoh subscriber all link.
- 100 Hz control loop (drift-free `clock_nanosleep` absolute deadlines).
- PPM frames: correct 14-byte framing, channel order (A,E,T,R,stab), checksums.
- `MocapFrame` decode round-trips with exact values (quaternion order x,y,z,w),
  byte-compatible with the `synapse_fbs` 0.1.1 wire format.

## Notes / TODO before flight

- **Waypoints are still the 100×100 m placeholder box** from the embedded
  parameter set (`fixed_wing_init_parameters` in `main.c`). Set these to the
  real course before flying.
- Confirm the **PPM channel order** (`map_output` in `main.c`) against the
  airframe/Tx — it currently follows the firmware's A,E,T,R,stabilizer layout.
- Live mocap reception must be validated on a machine with working zenoh
  transport and the qualisys bridge running.
