#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export RUST_LOG=DEBUG
# note, requires version 7
zenohd --config $SCRIPT_DIR/zenohd.json
