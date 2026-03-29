# Dynamic Notch Filter via BDShot eRPM

## Problem

Motor vibration shows up in the gyro signal at the motor's fundamental frequency
and its harmonics. Without filtering, this noise feeds into the PID and limits
how high gains can be tuned before oscillation. A dynamic notch filter tracks the
actual motor frequency in real time, so it can remove motor noise without the
phase lag of a wide static low-pass filter.

## Current state

The BDShot driver (`drivers/nxp_flexio_dshot/nxp_flexio_dshot.c`) already
decodes per-channel eRPM from ESC telemetry responses. The decoded values are
stored in `dshot_info[channel].erpm` but are not consumed by any application
code. Baudrate training (per-channel TCMP offset sweep) is implemented to handle
timing differences between ESC hardware (AM32 vs BLHeli32).

eRPM access paths that exist today:
- `up_bdshot_decode_erpm()` -- called internally from `sensor_sample_fetch`
- `up_bdshot_get_erpm()` -- exists but is dead code, not declared in any header
- `sensor_channel_get(dev, SENSOR_CHAN_RPM, val)` -- wired up but never called

None of these are called from the control loop.

## Where the notch filter fits

```
IMU (gyro) --> dynamic notch filter(s) --> rate_control PID --> mixer --> DShot
                       ^
                 eRPM from BDShot (per-channel, decoded each cycle)
```

The insertion point is between `cerebri_imu_stream_wait_next()` and
`cerebri_rate_controller_step()` in `main.c`. Filter the gyro vector before the
PID sees it.

## eRPM read timing

The BDShot response from frame N arrives before frame N+1 is triggered. The
control loop blocks on IMU data, then runs PID + motor output synchronously.
This means eRPM from the previous DShot frame is available at the top of the
next loop iteration -- one frame of latency (1.25 ms at 800 Hz), which is
negligible for notch filter tracking since motor speed changes on the order of
tens of milliseconds.

## Design options

### Option A: Single notch from average RPM (simplest)

Average the eRPM across all 4 motors and apply one biquad notch filter per gyro
axis (3 biquads total).

- Pros: Minimal compute, simple to implement, works well when all motors spin at
  similar speeds (hover, forward flight).
- Cons: During aggressive yaw or if one motor is significantly different, the
  averaged frequency misses the actual vibration source.

### Option B: Per-motor notch (4 notches per axis)

Apply a separate notch for each motor's eRPM on each gyro axis (12 biquads
total).

- Pros: Handles asymmetric motor speeds correctly. Each motor's vibration is
  independently tracked.
- Cons: 12 biquad evaluations per sample at 800 Hz. Each biquad is ~5 multiply-
  adds, so roughly 60 multiply-adds total -- well within budget on Cortex-M7 but
  worth measuring.

### Option C: Per-motor notch + first harmonic (8 notches per axis)

Same as Option B but add a second notch at 2x the fundamental for each motor
(24 biquads total).

- Pros: The first harmonic is often the strongest vibration source on some frames.
  Catches both fundamental and 2x.
- Cons: 24 biquads per sample. Still feasible on M7 but adds ~120 multiply-adds.
  Diminishing returns if the fundamental is the dominant source.

### Recommendation

Start with Option B (per-motor notch, no harmonics). It is the best balance of
correctness and simplicity. Option A can miss real vibration when motor speeds
diverge; Option C adds complexity for a problem that may not exist on this
airframe. Harmonics can be added later if bench testing shows they are needed.

## Implementation sketch

### 1. eRPM read

Add a function to read eRPM from the DShot device at the top of the main loop,
after IMU wait returns but before the PID runs.

```c
/* In main.c loop body, after cerebri_control_input_wait() */
float motor_freq_hz[4];
for (int i = 0; i < 4; i++) {
    int erpm;
    if (up_bdshot_get_erpm(dshot_dev, i, &erpm) == 0 && erpm > 0) {
        motor_freq_hz[i] = (float)erpm * MOTOR_POLES / 120.0f;
    }
    /* else keep previous frequency */
}
```

`up_bdshot_get_erpm` needs to be declared in the public header or replaced with
an equivalent API function. `MOTOR_POLES` is a hardware constant (typically 14
for 5" quad motors).

### 2. Biquad notch filter

A second-order IIR notch with configurable center frequency and bandwidth:

```c
struct biquad {
    float b0, b1, b2;  /* numerator coefficients */
    float a1, a2;       /* denominator coefficients (a0 = 1) */
    float x1, x2;       /* input delay line */
    float y1, y2;       /* output delay line */
};

void biquad_notch_update(struct biquad *f, float center_hz, float bandwidth_hz,
                         float sample_rate_hz);
float biquad_apply(struct biquad *f, float input);
```

Coefficients are recalculated only when `center_hz` changes by more than a
threshold (e.g. 5 Hz) to avoid unnecessary trig calls every iteration.

### 3. Filter bank

```c
struct notch_filter_bank {
    struct biquad notch[4][3];  /* [motor][axis] */
    float last_freq_hz[4];     /* for change detection */
};
```

Each gyro axis sample passes through 4 cascaded notch filters (one per motor):

```c
for (int motor = 0; motor < 4; motor++) {
    if (freq_changed(motor_freq_hz[motor], bank->last_freq_hz[motor])) {
        for (int axis = 0; axis < 3; axis++) {
            biquad_notch_update(&bank->notch[motor][axis],
                                motor_freq_hz[motor], notch_bw, 800.0f);
        }
        bank->last_freq_hz[motor] = motor_freq_hz[motor];
    }
}

gyro->x = apply_motor_notches(bank, 0, gyro->x);
gyro->y = apply_motor_notches(bank, 1, gyro->y);
gyro->z = apply_motor_notches(bank, 2, gyro->z);
```

### 4. Notch bandwidth

A Q factor of 5-8 (bandwidth = center/Q) is typical. Too narrow and the notch
doesn't catch frequency jitter; too wide and it adds phase lag. A reasonable
starting point is Q=6, giving ~17 Hz bandwidth at a 100 Hz center frequency.

### 5. Minimum frequency guard

Don't apply a notch below a minimum frequency (e.g. 50 Hz). At low RPM the eRPM
measurement is noisy and a very low-frequency notch would distort the actual
flight dynamics signal. When a motor reports eRPM below the threshold, skip its
notch (pass through unfiltered).

## Files to touch

| File | Change |
|---|---|
| `src/main.c` | Read eRPM, apply notch bank to gyro before PID |
| `src/notch_filter.c` (new) | Biquad notch implementation and filter bank |
| `src/notch_filter.h` (new) | Public API for notch filter bank |
| `include/.../nxp_flexio_dshot.h` | Declare eRPM access function if needed |
| `CMakeLists.txt` | Add `notch_filter.c` to build |

## Open questions

- What is the motor pole count for the target airframe?
- Should the notch be disabled during BDShot training (when eRPM is unavailable)?
  Likely yes -- just pass gyro through unfiltered until training completes.
- Is the ICM45686 hardware LPF (currently BW/16 for accel, BW/8 for gyro)
  sufficient as an anti-aliasing filter, or should a software LPF be added in
  addition to the notch?
- Should notch filter state be reset on arm/disarm transitions to avoid transient
  ringing from stale delay-line values?
