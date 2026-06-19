# cubs2 SIL — what the simulator must do

This describes the contract a flight-dynamics simulator must satisfy to fly the
`cubs2` fixed-wing controller in software-in-the-loop (`native_sim`).

```
            UDP 4245  SimInput (pose + RC)
  simulator ───────────────────────────────▶ cubs2 native_sim (controller, 100 Hz)
            ◀───────────────────────────────
            UDP 4243  FlightSnapshot (commanded RC / state)
```

The controller is the same code that flies on hardware. The simulator replaces
the motion-capture rig and the airframe: it provides the vehicle **pose** and
consumes the controller's **stick commands**, integrating its own dynamics.

## Transport

- UDP, host `127.0.0.1` (`CONFIG_CUBS2_SITL_HOST`).
- **Inbound** (sim → controller): `SimInput` FlatBuffer to **port 4245**.
- **Outbound** (controller → sim): `FlightSnapshot` FlatBuffer from **port 4243**;
  `MotorOutput` from 4244 (multirotor only — **not used by the fixed-wing cub**).
- Raw FlatBuffer bytes, no length prefix or framing (one datagram = one message).
- The controller keeps only the **latest** `SimInput` (double-buffered); stale or
  dropped packets are fine. Outbound messages are emitted whenever the controller
  produces a new cycle.

Schemas: [`synapse_sil.fbs`](../../src/synapse_sil.fbs),
[`synapse_topics.fbs`](../../src/synapse_topics.fbs),
[`synapse_mocap.fbs`](../../src/synapse_mocap.fbs). Generated `.bfbs` for
reflection-based codecs land in `<build>/generated/flatbuffers/`.

## Inbound: `SimInput` (what the sim sends, ≥100 Hz)

```
table SimInput {
  gyro:Vec3f;                      // unused by fixed-wing (set 0)
  accel:Vec3f;                     // unused by fixed-wing (set 0)
  rc:RcChannels16;                 // pilot/arm channels (optional for auto)
  rc_link_quality:ubyte;
  rc_valid:bool;
  imu_valid:bool;
  mocap:MocapRigidBodySample;      // REQUIRED: simulated vehicle pose
}
```

`MocapRigidBodySample` (same type the real Qualisys bridge emits):

```
struct MocapRigidBodySample {
  id:int;                          // any non-zero id, e.g. 1
  position:Vec3f { x, y, z };      // meters, world frame (see below)
  attitude:Quaternionf { x, y, z, w };  // unit quaternion, body orientation
  residual:float;                  // 0
  tracking_valid:bool;             // MUST be true for the pose to be used
}
```

**Only `mocap` matters for the fixed-wing controller.** The controller derives
all velocities, rates, flight-path angle, etc. by finite-differencing position
and attitude internally — exactly as the Python `cub_tecs_ros_xtrack` node did.
`gyro`/`accel` are ignored when a valid mocap pose is present.

### Frame and units
- **Position** in meters in the same world frame as the waypoints
  (`fixed_wing_init_parameters` in [`src/main.c`](../../src/main.c)).
  **`z` is altitude, positive up.** Start the aircraft on the ground with
  `z` below `takeoffAltitude` (0.4 m) — the takeoff→airborne transition is
  **edge-triggered** on `z` crossing that threshold upward, so the sim must
  publish the on-ground state first.
- **Attitude**: unit quaternion of the body in the world frame, components
  ordered `x, y, z, w`. The controller converts to roll/pitch/yaw with the
  standard formula (`quat_to_euler` in `src/main.c`); match the convention the
  Qualisys bridge uses (right-handed, body-to-world).

## Outbound: `FlightSnapshot` (what the sim reads, port 4243)

```
table FlightSnapshot {
  gyro; accel;
  rc:RcChannels16;                 // <-- the commanded actuator outputs
  status; attitude; attitude_desired; rate_desired; rate_cmd;
}
```

Read **`rc`** — the controller's commanded sticks as PWM microseconds:

| Channel | Surface | Encoding |
|---------|---------|----------|
| `ch0` | aileron | 1000–2000, 1500 = center |
| `ch1` | elevator | 1000–2000, 1500 = center (sign already applied) |
| `ch2` | throttle | 1000–2000, 1000 = idle, 2000 = full |
| `ch3` | rudder | 1000–2000, 1500 = center |
| `ch4` | stabilizer/mode | ~1900 (passthrough) |

Convert to physical inputs for your plant, e.g.
`aileron_cmd = (ch0 - 1500) / 500` ∈ [−1, 1], `throttle = (ch2 - 1000) / 1000`
∈ [0, 1], and scale by your control-surface limits.

## Timing

- The control loop **free-runs at 100 Hz** and integrates at a fixed
  `dt = 0.01 s`; it reads whatever `SimInput` is latest each tick (it does not
  block on or sync to the sim).
- Publish `SimInput` at **≥100 Hz** so each tick sees a fresh pose. Repeating an
  identical position makes the controller's finite-difference velocity read zero
  for that tick.
- Run your dynamics at 100 Hz to stay matched to the controller's `dt`. The
  transport is best-effort (not lock-step); for repeatable runs keep the sim and
  controller on the same 100 Hz cadence.

## Minimal simulator loop

```
state = on_ground(position=wp1_xy, z=0, level attitude)
every 10 ms:
    # 1. publish pose
    SimInput.mocap = { id:1, position:state.pos, attitude:state.quat,
                       tracking_valid:true }
    SimInput.rc_valid = true            # plus an armed/mode rc if you model it
    send SimInput  -> udp 127.0.0.1:4245

    # 2. read latest command (non-blocking)
    if FlightSnapshot available on udp :4243:
        ail = (rc.ch0-1500)/500; ele = (rc.ch1-1500)/500
        thr = (rc.ch2-1000)/1000; rud = (rc.ch3-1500)/500

    # 3. integrate your 6-DOF fixed-wing dynamics with (ail,ele,thr,rud), dt=0.01
    state = step(state, ail, ele, thr, rud, dt=0.01)
```

## Build and run the SIL controller

```sh
# flatcc must be on PATH (see ../../host/build.sh for a vendored copy)
west build -b native_sim <ws>/cerebri/cubs2 -d <build>
<build>/zephyr/zephyr.exe          # listens on 4245, emits on 4243
```

Ports/host are set in [`../../prj.conf`](../../prj.conf)
(`CONFIG_CUBS2_SITL_RX_PORT`, `..._TX_FLIGHT_SNAPSHOT_PORT`,
`..._TX_MOTOR_OUTPUT_PORT`, `CONFIG_CUBS2_SITL_HOST`).

Any standard FlatBuffers builder works for `SimInput` (the SITL reader handles
both vtable layouts); the `.fbs`/`.bfbs` files above are the single source of
truth for the wire format.
