#!/bin/bash
ros2 topic pub /joy sensor_msgs/Joy '{axes: [0, 1.0, 0, 0, 0, 0], buttons: [1, 0]}' &
sleep 2
ros2 run ros_gz_bridge parameter_bridge /joy@sensor_msgs/msg/Joy@gz.msgs.Joy
