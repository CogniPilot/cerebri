#!/bin/bash
west init -l .
west update .
west build -- -DCONFIG_COVERAGE=y

./build/zephyr/zephyr.elf &
gz sim quad.sdf -s -r &

ros2 topic pub /joy sensor_msgs/Joy '{axes: [0, 1.0, 0, 0, 0, 0], buttons: [1, 0]}' &
ros2 run joy joy_node &
sleep 2
ros2 run ros_gz_bridge parameter_bridge /joy@sensor_msgs/msg/Joy@gz.msgs.Joy &

# code coverage
lcov --capture --directory ./ --output-file lcov.info -q --rc lcov_branch_coverage=1
genhtml lcov.info --output-directory lcov_html -q --ignore-errors source --branch-coverage --highlight --legend

pkill ros