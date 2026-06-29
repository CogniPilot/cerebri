#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Build the native cubs2_host executable.
#
#   - fetches & builds flatcc (FlatBuffer C codegen + runtime) if missing
#   - generates FlatBuffer readers from ../src/*.fbs
#   - compiles the legacy host control loop and serial output
#
# Output: build/cubs2_host
set -euo pipefail

HOST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$HOST_DIR/../src" && pwd)"
TP="$HOST_DIR/third_party"
BUILD="$HOST_DIR/build"
GEN="$BUILD/gen"
JOBS="$(nproc 2>/dev/null || echo 4)"

mkdir -p "$TP" "$GEN"

# --- flatcc -------------------------------------------------------------
FLATCC_DIR="$TP/flatcc"
if [ ! -x "$FLATCC_DIR/bin/flatcc" ]; then
	echo ">> building flatcc"
	rm -rf "$FLATCC_DIR"
	git clone --depth 1 https://github.com/dvidelabs/flatcc.git "$FLATCC_DIR"
	( cd "$FLATCC_DIR" && scripts/build.sh )
fi
FLATCC="$FLATCC_DIR/bin/flatcc"

# --- generate FlatBuffer readers ---------------------------------------
echo ">> generating FlatBuffer readers"
"$FLATCC" -a -I"$SRC_DIR" -o"$GEN" "$SRC_DIR/synapse_topics.fbs"
"$FLATCC" -a -I"$SRC_DIR" -o"$GEN" "$SRC_DIR/synapse_mocap.fbs"

# --- compile ------------------------------------------------------------
echo ">> compiling cubs2_host"
cc -O2 -std=c11 -Wall -Wextra -Wno-unused-parameter \
	-D_DEFAULT_SOURCE \
	-I"$GEN" \
	-I"$SRC_DIR/generated_fixed_wing" \
	-I"$FLATCC_DIR/include" \
	-I"$SRC_DIR" \
	-I"$HOST_DIR" \
	-I"$HOST_DIR/compat" \
	"$HOST_DIR/main.c" \
	"$HOST_DIR/ppm_serial.c" \
	"$HOST_DIR/mocap_sub.c" \
	"$SRC_DIR/topic_flatbuffer.c" \
	"$SRC_DIR/generated_fixed_wing/CubControl_FixedWingOuterLoop.c" \
	"$FLATCC_DIR/lib/libflatccrt.a" \
	-lm \
	-o "$BUILD/cubs2_host"

echo ">> built $BUILD/cubs2_host"
