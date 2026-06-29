# cubs2_host

Legacy native Linux fixed-wing controller smoke test.

This path no longer owns live mocap or Zenoh transport. The project is moving
to the Zephyr `native_sim` executable plus csyn for FlatBuffer topic routing.

Use this host binary only when you need a simple controller-to-serial smoke
test. Without live mocap it emits failsafe PPM frames: throttle low and sticks
centered.

## Build

```sh
./build.sh
```

The build fetches flatcc if needed, generates FlatBuffer readers, and compiles
`build/cubs2_host`.

## Run

```sh
./build/cubs2_host /dev/ttyACM0
```

`CUBS2_PPM_DEVICE` can still select the serial device when no argv device is
provided.

For live RC, mocap, and controller-output routing, use the Zephyr `native_sim`
build with the csyn Zephyr module or csyn host bridge.
