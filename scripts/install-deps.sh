#!/usr/bin/env bash
set -euo pipefail

apt-get update
apt-get install -y \
  python3-pyudev \
  ros-humble-controller-interface \
  ros-humble-depthai-descriptions \
  ros-humble-dynamixel-interfaces \
  ros-humble-dynamixel-sdk \
  ros-humble-generate-parameter-library \
  ros-humble-image-geometry \
  ros-humble-gz-ros2-control \
  ros-humble-joint-state-publisher \
  ros-humble-laser-filters \
  ros-humble-moveit-msgs \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-servo \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-msgs \
  ros-humble-realtime-tools \
  ros-humble-robot-localization \
  ros-humble-ros2-control \
  ros-humble-realsense2-camera \
  ros-humble-ros2-controllers \
  ros-humble-slam-toolbox
