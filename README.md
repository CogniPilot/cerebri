# cerebri

`cerebri` contains maintained vehicle and platform examples that show how to
use CogniPilot infrastructure to build Zephyr RTOS applications.

Each vehicle lives in its own top-level folder and owns its own:
- `west.yml`
- `spec/`
- Zephyr build files
- application source, drivers, and subsystems

This means the repository root is not the application root, and there is
intentionally no root `west.yml`.

Current platform:
- `rdd2/`

For a given vehicle, treat `<vehicle>/spec/` as the source of truth for that
vehicle's policies and architecture. Start with
[rdd2/spec/README.md](/home/jgoppert/cognipilot/ws/cerebri/rdd2/spec/README.md)
and
[rdd2/README.md](/home/jgoppert/cognipilot/ws/cerebri/rdd2/README.md)
for the active `rdd2` platform.

## West Layout

Clone this repository as `cerebri` inside a west workspace, then initialize
west against the vehicle manifest you want to use. For `rdd2`:

```sh
git clone <repo-url> /tmp/cerebri-ws/cerebri
cd /tmp/cerebri-ws
west init -l cerebri --mf rdd2/west.yml
west update
west build -b mr_vmu_tropic/mimxrt1064 cerebri/rdd2 -d build/rdd2
```

That layout keeps each vehicle self-contained while still letting one shared
workspace host Zephyr, modules, and multiple CogniPilot repositories.

## Policy

This repository structure follows the platform-local manifest approach described
in CEP-0002. This README explains the practical layout used here; read the CEP
for the broader rationale and policy:

- [CEP-0002: Platform Repository Restructuring](https://github.com/CogniPilot/CEP/blob/main/CEP-0002.md)
