#!/bin/bash
sudo mkdir -p /opt/ws_ros_gz
sudo chown -R installer:installer /opt/ws_ros_gz
mkdir -p /opt/ws_ros_gz/src
cd /opt/ws_ros_gz/src
git clone -b ros2 https://github.com/gazebosim/ros_gz.git
cd /opt/ws_ros_gz
source /opt/ros/humble/setup.bash
colcon build

