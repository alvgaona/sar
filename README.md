# Search & Rescue

Search & Rescue (SAR) robotics project at the Center of Automation and
Robotics (CAR), Universidad Politécnica de Madrid. A Husarion ROSbot XL —
or our custom SAR platform sharing the same kinematics — explores an
indoor environment autonomously: SLAM builds the map, frontier-based
exploration drives the robot, Nav2 handles navigation, and an ArUco
detector flags markers placed in the scene.

ROS 2 Humble. Tested on macOS (Apple Silicon) via Pixi and on Ubuntu
22.04. Simulation uses Gazebo (`gz-sim`) through `ros_gz`.

## Workspace

Packages live at the repo root (not under `src/`).

| Package | Purpose |
| --- | --- |
| [`sar_bringup`](sar_bringup) | Top-level launches that compose sim/real stacks |
| [`sar_control`](sar_control) | ros2_control hardware interface (Arduino over USB serial) |
| [`sar_description`](sar_description) | URDF/xacro for real + sim, single launch |
| [`sar_exploration`](sar_exploration) | Frontier exploration (SLAM + Nav2 + explore_lite) |
| [`sar_gazebo`](sar_gazebo) | Gazebo worlds, models, and the sim launch |
| [`sar_mission`](sar_mission) | Mission state machine (pause exploration on marker) |
| [`sar_msgs`](sar_msgs) | Custom ROS messages |
| [`sar_perception`](sar_perception) | ArUco detector + optional RealSense driver |
| [`sar_slam`](sar_slam) | `slam_toolbox` async wrapper |
| [`sar_utils`](sar_utils) | Joystick / keyboard teleop |

External dependencies are pulled in as git submodules:
`rosbot_ros`, `husarion_components_description`, `husarion_controllers`,
`husarion_gz_worlds`, `m-explore-ros2`, `rplidar_ros`.

## Setup

Always clone with submodules:

```bash
git clone --recurse-submodules https://github.com/alvgaona/sar.git
# or, if already cloned:
git submodule update --init --recursive
```

### Pixi (macOS / Linux)

Self-contained conda env using the `robostack-humble` channel; no system
ROS install needed.

```bash
pixi install                          # provision the env (one-time)
pixi shell                            # enter the env (sources install/setup.sh)
pixi run build                        # build everything
pixi run build-pkg sar_gazebo         # build a single package and its deps
pixi run clean                        # wipe build/install/log
```

### Ubuntu 22.04 (native ROS Humble)

Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
first, then:

```bash
sudo ./scripts/install-deps.sh
colcon build --symlink-install
source install/setup.bash
```

### Docker (any host)

Brings up a desktop-VNC ROS Humble container (`alvgaona/ros:humble-desktop-vnc`)
mounting the workspace at `/root/sar_ws`. VNC is on `localhost:5901`.

```bash
docker compose up -d
docker compose exec sar bash

# inside the container, on first use:
./scripts/install-deps.sh
colcon build --symlink-install
source install/setup.sh
```

## Run

Composed stacks via `sar_bringup`:

```bash
ros2 launch sar_bringup sim.launch.py     # Gazebo + SLAM + exploration + perception
ros2 launch sar_bringup real.launch.py    # sar_control + SLAM + exploration + perception
```

Or piece-by-piece (each package's README has more):

```bash
ros2 launch sar_gazebo gazebo.launch.py                       # sim only
ros2 launch sar_slam slam.launch.py use_sim_time:=true
ros2 launch sar_exploration explore.launch.py use_sim_time:=true
ros2 launch sar_perception perception.launch.py
ros2 launch sar_control control.launch.py [mock:=true]        # real robot
```

## License

[Apache 2.0](LICENSE)
