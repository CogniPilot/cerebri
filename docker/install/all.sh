#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
	then echo "Please run using sudo"
	exit
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
$SCRIPT_DIR/base.sh
$SCRIPT_DIR/zephyr.sh
$SCRIPT_DIR/ros.sh
$SCRIPT_DIR/gazebo.sh
$SCRIPT_DIR/zenoh.sh
$SCRIPT_DIR/extra.sh
