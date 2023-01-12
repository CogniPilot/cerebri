#!/bin/bash
set -e
ROS_VERSION="humble"

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get -y update
sudo apt-get -y upgrade
sudo DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
	ros-${ROS_VERSION}-desktop-full \
	ros-${ROS_VERSION}-cyclonedds \
	ros-${ROS_VERSION}-rmw-cyclonedds-cpp \
	ros-${ROS_VERSION}-ros-gz-bridge \
	ros-${ROS_VERSION}-gps-msgs \
	python3-colcon-ros
