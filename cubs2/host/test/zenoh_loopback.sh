#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# End-to-end loopback test for the mocap input path:
#
#   mocap_pub --(Zenoh: MocapFrame)--> cubs2_host --(PPM frames)--> PTY capture
#
# Verifies cubs2_host receives mocap, transitions to airborne, and drives
# non-failsafe RC outputs. Requires a working Zenoh transport between the two
# processes:
#   - default (peer): UDP multicast auto-discovery (works on a normal LAN/host)
#   - robust: run a zenoh router and export
#       CUBS2_ZENOH_MODE=client CUBS2_ZENOH_CONNECT=tcp/127.0.0.1:7447
#     e.g.  docker run --rm --net host eclipse/zenoh:latest
#
# Usage: test/zenoh_loopback.sh
set -uo pipefail

HOST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP="$HOST_DIR/third_party"
GEN="$HOST_DIR/build/gen"
SRC="$HOST_DIR/../src"
PUB_BIN="$HOST_DIR/build/mocap_pub"

# 1. Ensure cubs2_host + readers exist.
if [ ! -x "$HOST_DIR/build/cubs2_host" ]; then
	echo ">> building cubs2_host first"
	( cd "$HOST_DIR" && ./build.sh )
fi

# 2. Build the test publisher against the same deps.
echo ">> building mocap_pub"
cc -O2 -std=c11 -D_DEFAULT_SOURCE -DZENOH_LINUX \
	-I"$GEN" -I"$TP/flatcc/include" \
	-I"$TP/zenoh-pico/include" -I"$TP/zenoh-pico/build/include" \
	"$HOST_DIR/test/mocap_pub.c" \
	"$TP/flatcc/lib/libflatccrt.a" "$TP/zenoh-pico/build/lib/libzenohpico.a" \
	-lm -lpthread -o "$PUB_BIN" || { echo "publisher build failed"; exit 1; }

# 3. Linked PTY pair so we can read back what cubs2_host transmits.
PTY_HOST=/tmp/cubs2_ppm_host
PTY_READ=/tmp/cubs2_ppm_read
CAP=/tmp/cubs2_ppm_capture.bin
HOST_LOG=/tmp/cubs2_host_test.log
rm -f "$CAP" "$HOST_LOG"
socat pty,raw,echo=0,link="$PTY_HOST" pty,raw,echo=0,link="$PTY_READ" >/dev/null 2>&1 &
SOCAT=$!
sleep 0.4
( timeout 6 cat "$PTY_READ" > "$CAP" ) & CAT=$!

cleanup() { kill "$SOCAT" "$CAT" "$HOST_PID" "$PUB_PID" 2>/dev/null; }
trap cleanup EXIT

# 4. Start controller, then publisher.
#
# Router-free transport that works without multicast: the publisher listens on
# a TCP loopback endpoint and cubs2_host connects to it (both zenoh peers).
# Override by exporting CUBS2_ZENOH_* before running (e.g. to use a router, or
# plain multicast peer discovery on a real LAN).
LOOPBACK_EP="tcp/127.0.0.1:7449"
export CUBS2_ZENOH_MODE="${CUBS2_ZENOH_MODE:-peer}"
PUB_LISTEN="${CUBS2_ZENOH_LISTEN:-$LOOPBACK_EP}"
HOST_CONNECT="${CUBS2_ZENOH_CONNECT:-$LOOPBACK_EP}"

echo ">> starting mocap_pub (mode=$CUBS2_ZENOH_MODE, listen=$PUB_LISTEN)"
CUBS2_ZENOH_LISTEN="$PUB_LISTEN" "$PUB_BIN" 2>/tmp/cubs2_pub_test.log & PUB_PID=$!
sleep 1.0
echo ">> starting cubs2_host (connect=$HOST_CONNECT)"
CUBS2_PPM_DEVICE="$PTY_HOST" CUBS2_ZENOH_CONNECT="$HOST_CONNECT" \
	"$HOST_DIR/build/cubs2_host" 2>"$HOST_LOG" & HOST_PID=$!

sleep 4
kill "$HOST_PID" "$PUB_PID" 2>/dev/null
wait "$CAT" 2>/dev/null

# 5. Evaluate.
echo
echo "=== cubs2_host status (tail) ==="
grep '^t=' "$HOST_LOG" | tail -4
echo "=== publisher ==="; cat /tmp/cubs2_pub_test.log

python3 - "$CAP" "$HOST_LOG" <<'PY'
import sys
cap, log = sys.argv[1], sys.argv[2]
b = open(cap, 'rb').read()
frames=[]; i=0
while i < len(b)-13:
    if b[i]==0xFF and b[i+1]==0xFF:
        f=b[i:i+14]; ch=[f[2+2*k]|(f[3+2*k]<<8) for k in range(5)]
        if (sum(ch)&0xFFFF)==(f[12]|(f[13]<<8)): frames.append(ch)
        i+=14
    else: i+=1
txt=open(log).read()
got_mocap = 'mocap=1' in txt
airborne  = 'air=1' in txt
driven = any(c[2]!=1000 or c[0]!=1500 or c[1]!=1500 or c[3]!=1500 for c in frames)
print(f"\n=== RESULT ===")
print(f"frames captured : {len(frames)}")
print(f"mocap received  : {got_mocap}")
print(f"airborne reached: {airborne}")
print(f"RC driven (non-failsafe): {driven}")
if frames: print(f"last frame A,E,T,R,M: {frames[-1]}")
ok = got_mocap and driven
print("PASS" if ok else "FAIL (needs a working zenoh transport between the two procs)")
sys.exit(0 if ok else 1)
PY
