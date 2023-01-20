#!/usr/bin/env bash
ros2 run joy joy_node &
sleep 2
ros2 run ros_gz_bridge parameter_bridge /joy@sensor_msgs/msg/Joy@gz.msgs.Joy
