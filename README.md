# Search &amp; Rescue

A Search &amp; Rescue (SAR) project at the facilities of the Center of Automatic and Robotics (CAR) at
Universidad Politecnica de Madrid.

## Installation

### Pixi (macOS / Development)

```bash
pixi install
pixi shell
```

### Docker (Linux / Testing)

Build and start the container:

```bash
docker compose up -d
docker compose exec sar bash
```

Inside the container, install dependencies and build:

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

[MIT](LICENSE.md)

