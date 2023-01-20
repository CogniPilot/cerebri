#!/bin/bash
set -e
set -x

echo whoami: `whoami`
echo pwd: `pwd`

ls -al

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

CEREBRI_DIR="$SCRIPT_DIR/.."
cd $CEREBRI_DIR
ls -al
echo pwd: `pwd`

if [ -d "$CEREBRI_DIR/../.west" ]; then
    echo "West repo already initialized"
else
    echo "Initialiizing west repo"
    west init -l .
fi
west update .
west build -p -- -DCONFIG_COVERAGE=y

ros2 run joy joy_node &
sleep 1
ros2 run ros_gz_bridge parameter_bridge /joy@sensor_msgs/msg/Joy@gz.msgs.Joy &

gz sim quad.sdf -s -r &
./build/zephyr/zephyr.elf &

ros2 topic pub -r 1 -t 2 /joy sensor_msgs/Joy '{axes: [0, 1.0, 0, 0, 0, 0], buttons: [1, 0]}'
ros2 topic pub -r 1 -t 2 /joy sensor_msgs/Joy '{axes: [0, 1.0, 0, 0, 0, 0], buttons: [0, 1]}'
pkill -9 zephyr
pkill -9 ros
pkill -9 joy
pkill -9 ruby
pkill -9 parameter_bridg

# code coverage
lcov --capture --directory ./ --output-file lcov.info -q --rc lcov_branch_coverage=1
genhtml lcov.info --output-directory lcov_html -q --ignore-errors source --branch-coverage --highlight --legend
