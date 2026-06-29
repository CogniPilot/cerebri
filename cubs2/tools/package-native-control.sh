#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Package the native_sim controls runtime for copying to another computer.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CUBS2_DIR=$(cd -- "${SCRIPT_DIR}/.." && pwd)
REPO_DIR=$(cd -- "${CUBS2_DIR}/.." && pwd)
BUILD_DIR=${1:-"${REPO_DIR}/build/cubs2-native-sim"}
OUT_DIR=${2:-"${REPO_DIR}/build/cubs2-native-control-package"}
BRIDGE_MANIFEST="${CUBS2_DIR}/tools/csyn-zephyr-bridge/Cargo.toml"
BRIDGE_DIR="${CUBS2_DIR}/tools/csyn-zephyr-bridge"
ZEPHYR_EXE="${BUILD_DIR}/zephyr/zephyr.exe"
BRIDGE_EXE="${BRIDGE_DIR}/target/release/cubs2-csyn-zephyr-bridge"

if [[ ! -x "${ZEPHYR_EXE}" ]]; then
	echo "missing ${ZEPHYR_EXE}" >&2
	echo "build it first, for example:" >&2
	echo "  west build -b native_sim cubs2 -d ${BUILD_DIR}" >&2
	exit 1
fi

cargo build --release --manifest-path "${BRIDGE_MANIFEST}"

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
cp "${ZEPHYR_EXE}" "${OUT_DIR}/zephyr.exe"
cp "${BRIDGE_EXE}" "${OUT_DIR}/cubs2-csyn-zephyr-bridge"

cat >"${OUT_DIR}/run-controls.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

CONNECT=${CSYN_CONNECT:-tcp/127.0.0.1:7447}

./cubs2-csyn-zephyr-bridge --connect "${CONNECT}" &
BRIDGE_PID=$!
trap 'kill "${BRIDGE_PID}" 2>/dev/null || true' EXIT INT TERM

./zephyr.exe
EOF
chmod +x "${OUT_DIR}/run-controls.sh"

tar -C "$(dirname "${OUT_DIR}")" -czf "${OUT_DIR}.tar.gz" "$(basename "${OUT_DIR}")"

echo "packaged controls runtime:"
echo "  ${OUT_DIR}"
echo "  ${OUT_DIR}.tar.gz"
