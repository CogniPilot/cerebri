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
MANUAL_BRIDGE_DIR=${SYNAPSE_MANUAL_CONTROL_BRIDGE_DIR:-"${REPO_DIR}/../../../synapse_manual_control_bridge"}
MANUAL_BRIDGE_MANIFEST="${MANUAL_BRIDGE_DIR}/Cargo.toml"
PPM_BRIDGE_DIR=${SYNAPSE_PPM_BRIDGE_DIR:-"${REPO_DIR}/../../../synapse_ppm_bridge"}
PPM_BRIDGE_MANIFEST="${PPM_BRIDGE_DIR}/Cargo.toml"
ZEPHYR_EXE="${BUILD_DIR}/zephyr/zephyr.exe"
BRIDGE_EXE="${BRIDGE_DIR}/target/release/cubs2-csyn-zephyr-bridge"
MANUAL_BRIDGE_EXE="${MANUAL_BRIDGE_DIR}/target/release/synapse-manual-control-bridge"
PPM_BRIDGE_EXE="${PPM_BRIDGE_DIR}/target/release/synapse-ppm-bridge"

if [[ ! -x "${ZEPHYR_EXE}" ]]; then
	echo "missing ${ZEPHYR_EXE}" >&2
	echo "build it first, for example:" >&2
	echo "  west build -b native_sim cubs2 -d ${BUILD_DIR}" >&2
	exit 1
fi

cargo build --release --manifest-path "${BRIDGE_MANIFEST}"
if [[ ! -f "${MANUAL_BRIDGE_MANIFEST}" ]]; then
	echo "missing ${MANUAL_BRIDGE_MANIFEST}" >&2
	echo "set SYNAPSE_MANUAL_CONTROL_BRIDGE_DIR to package the joystick Zenoh node" >&2
	exit 1
fi
cargo build --release --manifest-path "${MANUAL_BRIDGE_MANIFEST}" --bin synapse-manual-control-bridge
if [[ ! -f "${PPM_BRIDGE_MANIFEST}" ]]; then
	echo "missing ${PPM_BRIDGE_MANIFEST}" >&2
	echo "set SYNAPSE_PPM_BRIDGE_DIR to package the PPM serial Zenoh node" >&2
	exit 1
fi
cargo build --release --manifest-path "${PPM_BRIDGE_MANIFEST}" --bin synapse-ppm-bridge

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
cp "${ZEPHYR_EXE}" "${OUT_DIR}/zephyr.exe"
cp "${BRIDGE_EXE}" "${OUT_DIR}/cubs2-csyn-zephyr-bridge"
cp "${MANUAL_BRIDGE_EXE}" "${OUT_DIR}/synapse-manual-control-bridge"
cp "${PPM_BRIDGE_EXE}" "${OUT_DIR}/synapse-ppm-bridge"

cat >"${OUT_DIR}/run-controls.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

CONNECT=${CSYN_CONNECT:-udp/192.168.10.2:7447}
MANUAL_CONTROL_DEVICE=${MANUAL_CONTROL_DEVICE:-/dev/input/js0}
MANUAL_CONTROL_TOPIC=${MANUAL_CONTROL_TOPIC:-synapse/manual_control}
START_MANUAL_CONTROL=${START_MANUAL_CONTROL:-auto}
PPM_SERIAL_DEVICE=${PPM_SERIAL_DEVICE:-/dev/ttyACM0}
PPM_TOPIC=${PPM_TOPIC:-synapse/manual_control}
PPM_CONTROL_OUTPUT_TOPIC=${PPM_CONTROL_OUTPUT_TOPIC:-synapse/control_output}
PPM_BAUD_RATE=${PPM_BAUD_RATE:-57600}
# CUBS2 antenna path observed on the vehicle expects TAERM PPM slot order:
# throttle, aileron, elevator, rudder, mode. The bridge base order already
# matches that, so leave it identity by default.
PPM_CHANNEL_MAP=${PPM_CHANNEL_MAP:-0,1,2,3,4}
START_PPM_BRIDGE=${START_PPM_BRIDGE:-auto}

PIDS=()
cleanup() {
	for pid in "${PIDS[@]}"; do
		kill "${pid}" 2>/dev/null || true
	done
}
trap cleanup EXIT INT TERM

manual_control_watch() {
	while true; do
		if [[ ! -e "${MANUAL_CONTROL_DEVICE}" ]]; then
			echo "waiting for manual-control device ${MANUAL_CONTROL_DEVICE}" >&2
			while [[ ! -e "${MANUAL_CONTROL_DEVICE}" ]]; do
				sleep 1
			done
		fi

		echo "starting manual-control bridge on ${MANUAL_CONTROL_DEVICE}" >&2
		./synapse-manual-control-bridge \
			--device "${MANUAL_CONTROL_DEVICE}" \
			--zenoh-connect "${CONNECT}" \
			--topic "${MANUAL_CONTROL_TOPIC}" || true

		echo "manual-control bridge stopped; waiting before retry" >&2
		sleep 1
	done
}

ppm_bridge_watch() {
	while true; do
		if [[ ! -e "${PPM_SERIAL_DEVICE}" ]]; then
			echo "waiting for PPM serial device ${PPM_SERIAL_DEVICE}" >&2
			while [[ ! -e "${PPM_SERIAL_DEVICE}" ]]; do
				sleep 1
			done
		fi

		echo "starting PPM bridge on ${PPM_SERIAL_DEVICE} manual=${PPM_TOPIC} control_output=${PPM_CONTROL_OUTPUT_TOPIC}" >&2
		./synapse-ppm-bridge \
			--zenoh-connect "${CONNECT}" \
			--topic "${PPM_TOPIC}" \
			--control-output-topic "${PPM_CONTROL_OUTPUT_TOPIC}" \
			--serial-device "${PPM_SERIAL_DEVICE}" \
			--baud-rate "${PPM_BAUD_RATE}" \
			--channel-map "${PPM_CHANNEL_MAP}" || true

		echo "PPM bridge stopped; waiting before retry" >&2
		sleep 1
	done
}

./cubs2-csyn-zephyr-bridge --connect "${CONNECT}" &
PIDS+=("$!")

if [[ "${START_MANUAL_CONTROL}" == "1" || "${START_MANUAL_CONTROL}" == "auto" ]]; then
	manual_control_watch &
	PIDS+=("$!")
else
	echo "manual-control bridge disabled by START_MANUAL_CONTROL=${START_MANUAL_CONTROL}" >&2
fi

if [[ "${START_PPM_BRIDGE}" == "1" || "${START_PPM_BRIDGE}" == "auto" ]]; then
	ppm_bridge_watch &
	PIDS+=("$!")
else
	echo "PPM bridge disabled by START_PPM_BRIDGE=${START_PPM_BRIDGE}" >&2
fi

./zephyr.exe
EOF
chmod +x "${OUT_DIR}/run-controls.sh"

tar -C "$(dirname "${OUT_DIR}")" -czf "${OUT_DIR}.tar.gz" "$(basename "${OUT_DIR}")"

echo "packaged controls runtime:"
echo "  ${OUT_DIR}"
echo "  ${OUT_DIR}.tar.gz"
