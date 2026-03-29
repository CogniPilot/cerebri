# AGENTS.md

This file tells coding agents how to work in `cerebri` without drifting from
the repo's flight-stack, latency, and code-organization rules.

## Scope

- Treat active specs in `spec/README.md` as the source of truth.
- `ACCEPTED` specs are mandatory.
- `DRAFT` specs describe the current intended direction and should not be
  violated casually.
- If code and spec disagree, update the spec or stop and clarify before adding
  more drift.

## Required Reading Before Editing

Start with `spec/README.md`, then load only the relevant specs.

Minimum default set for flight-stack work:

- `spec/SPEC_0002_LATENCY_DRIVEN_ARCHITECTURE.md`
- `spec/SPEC_0004_TROPIC_HARDWARE_SCOPE.md`
- `spec/SPEC_0006_CODE_SIZE_AND_DEBUG_SHELL.md`

Read additional specs by area:

- Spec formatting or new specs: `SPEC_0000`
- Repo structure or module ownership: `SPEC_0001`
- Control behavior or mixer work: `SPEC_0003`
- GNSS work: `SPEC_0005`
- Topic schema or diagnostics payload work: `SPEC_0007`
- SD-card log format work: `SPEC_0008`

## Mandatory Workflow For Agents

1. Identify whether the change touches the flight hot path or only tooling/debug code.
2. Read the relevant specs before editing.
3. Keep the change as small and local as possible.
4. Preserve the one-hot-path-thread architecture.
5. Update specs in the same change if you introduce or change a project rule.
6. Run the narrowest relevant verification before claiming completion.
7. If you add net code, do a compression pass and remove avoidable scaffolding.

If a semantic or architectural change lacks a governing spec, add or amend one
before expanding the implementation.

## Flight-Stack Rules

- No dependency on the legacy `cerebri` module.
- No `double` in the control path.
- No PWM path.
- CRSF only for RC input in v1.
- DSHOT only for motor output in v1.
- `mr_vmu_tropic` is the only supported target board in v1.
- Keep the rate controller, mixer, and motor output path obvious and local.

## Latency Rules

- The main control loop is the only application hot-path thread.
- Do not add queues between RC, IMU, controller, mixer, and DSHOT.
- Do not add periodic printing or logging to the 1 kHz loop.
- Any non-essential background activity must stay low priority and out of the hot path.

## Code-Size And Module Rules

- Keep every hand-written `.c` file under 2000 lines.
- Split by responsibility before a file becomes large and mixed-purpose.
- Keep debug shell code separate from the control-loop implementation.
- `topic echo` and `topic hz` must reuse the singleton diagnostics thread in
  `src/topic_shell.c`.
- Do not spawn per-command helper threads.

## Verification

At minimum, run the relevant build:

```sh
west build -b mr_vmu_tropic/mimxrt1064 /home/jgoppert/cognipilot/ws/cerebri \
  -d /home/jgoppert/cognipilot/ws/cerebri/build
```

If you could not run hardware validation, say so explicitly.

## When To Update Specs

Update or add a spec when you introduce:

- a new architecture rule,
- a new control-path invariant,
- a new code-organization policy,
- a new supported hardware assumption,
- or a new debug/diagnostics rule that affects collaboration.

Do not leave those rules implicit in code only.

## Done Means

Do not say a change is done unless:

- the relevant specs were checked,
- the implementation follows the current active specs,
- the build passed or a blocker was stated,
- and any new project rule was recorded in `spec/`.
