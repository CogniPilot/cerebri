# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

#!/usr/bin/env python3
import json
import os
import sys
import yaml
import re

# Arguments: board name, build dir
if len(sys.argv) != 3:
    print("Usage: generate_launch.py <board_name> <build_dir>")
    sys.exit(1)

board_name = sys.argv[1]
build_dir = os.path.abspath(sys.argv[2])

elf_file = os.path.join(build_dir, "zephyr/zephyr.elf")
exe_file = os.path.join(build_dir, "zephyr/zephyr.exe")

# Load runner.yaml
runner_yaml_path = os.path.join(build_dir, "zephyr/runners.yaml")
if not os.path.isfile(runner_yaml_path):
    raise FileNotFoundError(f"runner.yaml not found at {runner_yaml_path}")

with open(runner_yaml_path, "r") as f:
    runner_data = yaml.safe_load(f)

# Determine the default debug runner
debug_runner = runner_data.get("debug-runner")
runner_args = runner_data.get("args", {}).get(debug_runner, [])

# VSCode launch.json path — project root (one level up from build)
workspace_root = os.path.dirname(build_dir)
vscode_launch = os.path.join(workspace_root, ".vscode", "launch.json")
os.makedirs(os.path.dirname(vscode_launch), exist_ok=True)

# Read existing launch.json and remove comments
if os.path.isfile(vscode_launch):
    with open(vscode_launch, "r") as f:
        content = f.read()

    # JSON fixups before loading
    # Remove // comments
    content = re.sub(r"^\s*//.*$", "", content, flags=re.MULTILINE)
    # Remove trailing commas before } or ]
    content = re.sub(r',(\s*[}\]])', r'\1', content)
    try:
        launch_data = json.loads(content)
    except json.JSONDecodeError:
        print(f"Warning: invalid JSON in {vscode_launch}, starting with empty config")
        launch_data = {}
else:
    launch_data = {}

# Ensure configurations list exists
if "configurations" not in launch_data or not isinstance(launch_data["configurations"], list):
    launch_data["configurations"] = []

# New Zephyr debug config
config_name = f"Debug Zephyr {board_name}"

if debug_runner == "native":
    update_fields = {
        "name": config_name,
        "type": "cppdbg",
        "request": "launch",
        "program": exe_file,
        "cwd": "${fileDirname}",
        "MIMode": "gdb",
        "setupCommands": [
            {
                "description": "Enable pretty-printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": True
            },
            {
                "description": "Set Disassembly Flavor to Intel",
                "text": "-gdb-set disassembly-flavor intel",
                "ignoreFailures": True
            }
        ]
    }
elif debug_runner == "jlink":
    # Extract GDB path
    gdb_path = runner_data.get("config", {}).get("gdb")
    if not gdb_path:
        raise ValueError("GDB path not found in runner.yaml")

    # Extract device from runner args
    device = None
    for arg in runner_args:
        if "--device=" in arg:
            device = arg.split("=", 1)[1]
            break
        elif "--target=" in arg:
            device = arg.split("=", 1)[1]
            break

    if device is None:
        raise ValueError(f"Could not find device in runner args for {debug_runner}")

    update_fields = {
        "name": config_name,
        "type": "cortex-debug",
        "request": "launch",
        "executable": elf_file,
        "servertype": debug_runner,
        "interface": "swd",
        "rtos": "Zephyr",
        "device": device,
        "gdbPath": gdb_path,
    }

# Look for existing config
for i, cfg in enumerate(launch_data["configurations"]):
    if cfg.get("name") == config_name:
        # Merge fields, keeping anything else (like svdFile)
        cfg.update(update_fields)
        launch_data["configurations"][i] = cfg
        break
else:
    # If it doesn’t exist, append new config
    new_cfg = {"name": config_name}
    new_cfg.update(update_fields)
    launch_data["configurations"].append(new_cfg)

# Write back launch.json (JSON only, comments will be lost)
with open(vscode_launch, "w") as f:
    json.dump(launch_data, f, indent=4)
