# Search &amp; Rescue

A Search &amp; Rescue (SAR) project at the facilities of the Center of Automatic and Robotics (CAR) at
Universidad Politecnica de Madrid.

## Installation

### Pixi (macOS / Ubuntu)

```bash
pixi install
pixi shell
colcon build --symlink-install
source install/setup.sh
```

### Ubuntu 22.04 (Native)

Install ROS 2 Humble, then install additional dependencies and build:

```bash
sudo ./scripts/install-deps.sh
colcon build --symlink-install
source install/setup.sh
```

### Docker (macOS / Ubuntu / Windows — amd64 / arm64)

```bash
docker compose up -d
docker compose exec sar bash
```

Inside the container:

```bash
./scripts/install-deps.sh
colcon build --symlink-install
source install/setup.sh
```

## Usage

### Simulation (Gazebo)

```bash
ros2 launch sar_gazebo gazebo.launch.py
ros2 launch sar_slam slam.launch.py use_sim_time:=true
ros2 launch sar_exploration explore.launch.py use_sim_time:=true
```

### Real Robot (ros2_control)

```bash
ros2 launch sar_control control.launch.py
```

Mock mode (no Arduino needed):

```bash
ros2 launch sar_control control.launch.py mock:=true
```

## License

[Apache 2.0](LICENSE)

