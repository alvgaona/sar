#!/usr/bin/env bash
set -euo pipefail

DISTRO="${ROS_DISTRO:?ROS_DISTRO is not set}"

apt-get update
apt-get install -y \
  python3-pyudev \
  ros-"${DISTRO}"-controller-interface \
  ros-"${DISTRO}"-depthai-descriptions \
  ros-"${DISTRO}"-dynamixel-interfaces \
  ros-"${DISTRO}"-dynamixel-sdk \
  ros-"${DISTRO}"-generate-parameter-library \
  ros-"${DISTRO}"-image-geometry \
  ros-"${DISTRO}"-gz-ros2-control \
  ros-"${DISTRO}"-joint-state-publisher \
  ros-"${DISTRO}"-laser-filters \
  ros-"${DISTRO}"-moveit-msgs \
  ros-"${DISTRO}"-moveit-ros-planning \
  ros-"${DISTRO}"-moveit-ros-planning-interface \
  ros-"${DISTRO}"-moveit-servo \
  ros-"${DISTRO}"-nav2-costmap-2d \
  ros-"${DISTRO}"-nav2-msgs \
  ros-"${DISTRO}"-nav2-bringup \
  ros-"${DISTRO}"-realtime-tools \
  ros-"${DISTRO}"-robot-localization \
  ros-"${DISTRO}"-ros2-control \
  ros-"${DISTRO}"-realsense2-camera \
  ros-"${DISTRO}"-ros2-controllers \
  ros-"${DISTRO}"-slam-toolbox
