# Cerebri Control Loop Safety Evaluation

## What's done well

- **Arm/disarm gating is solid.** Disarm on IMU failure, RC stale, or arm switch
  low. Requires throttle-low to arm. Controller reset on arm transition.
- **Motor output clamping.** `motor_to_dshot` clamps to `[DSHOT_MIN, DSHOT_MAX]`,
  and the mixer scales rate commands down to fit the throttle envelope rather
  than clipping individual motors.
- **IMU timeout.** The RTIO path has a 5 ms timeout; if it fires, `imu_ok` goes
  false and the loop disarms.
- **RC staleness check.** 100 ms window, checked every iteration.
- **Test mode is gated.** Shell motor tests bypass the PID but the main loop
  still runs, so if test mode is cleared normal control resumes.

## Issues

Roughly in order of severity.

### 1. No dt sanity bounds (High)

`main.c:63-65` guards `dt <= 0`, but there is no upper bound. If an IMU hiccup
delivers a stale timestamp, `dt` could be huge (e.g. 0.5 s). That single
iteration would produce a massive derivative spike and an integrator jump. The
`pid_step` D-term calculation (`rate_control.c:82`) divides by `dt`, so a tiny
positive `dt` (e.g. from a timestamp glitch) would also produce an enormous
derivative.

**Fix:** Clamp `dt` to a sane range, e.g. `[0.5 * nominal, 2.0 * nominal]`.
Outside that range, substitute the nominal value and optionally flag it.

### 2. No NaN/Inf propagation protection (High)

Nothing in the PID, mixer, or motor output checks for NaN or Inf. A single
corrupted IMU sample (e.g. `0.0/0.0` from a decode failure) or a bad `dt` would
propagate NaN through the entire chain: `pid_step` -> `integrator` (permanently
poisoned) -> mixer -> motor output. DShot would then send garbage or zero
indefinitely until reboot.

**Fix:** Check `isfinite()` on gyro values after IMU decode. If any axis is
non-finite, treat it as an IMU failure (`imu_ok = false`). Optionally also check
PID output.

### 3. No hardware watchdog (High)

There is no watchdog anywhere in the codebase. If the main thread hangs (kernel
panic, deadlock in RTIO, priority inversion), motors hold their last commanded
value indefinitely. The ESCs have their own timeout (typically ~1 s without a
DShot frame), but that is the only fallback.

**Fix:** Configure the hardware WDT and kick it each loop iteration. If the loop
stalls, the MCU resets.

### 4. No loop overrun detection (Medium)

There is no monitoring of whether the control loop completes within its 1.25 ms
budget. If something takes too long (e.g. a `LOG_INF` that blocks on UART, a
pathological RTIO decode), the loop just runs late with no indication. Combined
with the dt issue above, late iterations silently degrade control.

**Fix:** Timestamp the start and end of each iteration. Log a warning (or
increment a counter) if the loop exceeds the budget. Could also feed this into a
health status topic.

### 5. Test mode bypasses arm switch (Medium)

`cerebri_motor_test_get` (`main.c:67-70`) bypasses all arm/disarm logic -- it
calls `write_all(..., armed=true, test_mode=true)` unconditionally. The shell
commands `motor spin` and `motor raw` set test mode with no arm-switch check.
This is by design for bench testing, but there is no guard against accidentally
leaving test mode active during flight. If someone sets a test value via shell
and then the RC link goes live, test mode takes priority.

**Fix:** Consider requiring a separate "bench mode" flag or disabling test mode
when the arm switch transitions high.

### 6. Motor idle throttle is 0.0 (Medium)

`CEREBRI_MOTOR_IDLE_THROTTLE` is `0.0f` (`motor_output.h:10`). When armed, the
throttle command at zero stick is `0.0 + (0.0 * 1.0) = 0.0`, which maps to
`DSHOT_DISARMED`. This means at zero throttle the ESCs receive disarm, and the
PID has no authority to maintain attitude. Most flight controllers use a small
idle (3-7%) to keep props spinning when armed so the controller can respond.

This may be intentional for the current development phase, but in flight it
would mean losing yaw authority at low throttle.

### 7. No gyro saturation detection (Low)

If the gyro is physically saturated (e.g. the ICM45686 hits its full-scale range
limit), the PID receives clipped measurements and the integrator winds up
against the real rate. There is no check for values near the sensor's full-scale
range.

### 8. No motor output rate limiting (Low)

The DShot value can jump from `DSHOT_MIN` to `DSHOT_MAX` in a single iteration.
In practice the ESCs and physics limit how fast things change, but a slew-rate
limit on the motor output would prevent the PID from commanding instantaneous
full-throttle steps, especially on initial arm or during recovery from a large
disturbance.

## Summary

| Issue | Severity | Effort |
|---|---|---|
| dt unbounded | High | Small -- add a clamp |
| No NaN/Inf check | High | Small -- `isfinite()` on gyro + dt |
| No hardware watchdog | High | Medium -- configure WDT peripheral |
| No loop overrun detection | Medium | Small -- timestamp + counter |
| Test mode bypasses arm | Medium | Small -- policy decision |
| Idle throttle is 0 | Medium | Trivial -- change the constant |
| No gyro saturation detect | Low | Small -- compare against FSR |
| No motor slew-rate limit | Low | Small -- add per-channel rate limiter |

The highest-impact, lowest-effort wins are the dt clamp and `isfinite()` checks
-- those two close the door on the most likely paths to uncontrolled motor output
from a single bad sample.
