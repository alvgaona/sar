# sar_gazebo

Gazebo (`gz-sim`) assets and launch files for simulating the ROSbot XL in SAR
environments. Wraps `rosbot_gazebo`/`husarion_gz_worlds` so the same launch can
bring up a Husarion stock world or a project-local world (e.g. `office`).

`GZ_SIM_RESOURCE_PATH` is extended with both `sar_gazebo/models/` and
`husarion_gz_worlds/models/` at launch time, so `<include><uri>model://...</uri>`
resolves against either tree.

## Install & build

The workspace supports two environments. Pixi is the default on macOS (Apple
Silicon) and works on Linux too; APT is the conventional path on Ubuntu 22.04
with ROS 2 Humble installed system-wide.

Either way, clone with submodules — `rosbot_ros`, `husarion_gz_worlds`,
`husarion_components_description`, `husarion_controllers`, and `m-explore-ros2`
are pulled in as git submodules and `sar_gazebo` depends on them:

```bash
git clone --recurse-submodules https://github.com/alvgaona/sar.git
# or, if already cloned:
git submodule update --init --recursive
```

### Pixi (macOS / Linux)

Uses the `robostack-humble` conda channel; no system ROS install needed.

```bash
pixi install                          # provisions the toolchain + ROS deps
pixi run build                        # build the whole workspace
pixi run build-pkg sar_gazebo         # or just this package (and its deps)
pixi shell                            # activate; sources install/setup.sh
```

### Ubuntu 22.04 + APT (ROS 2 Humble)

Install ROS 2 Humble first (see the
[ROS 2 install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)),
then pull package dependencies and build with colcon:

```bash
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-rviz2 \
    ros-humble-xacro

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Packages live at the repo root (not under `src/`), so run `colcon build` from
the repo root. Use `--packages-select sar_gazebo` (plus its deps) to scope the
build.

## Running

```bash
# Pixi:  pixi shell
# APT:   source install/setup.bash
ros2 launch sar_gazebo gazebo.launch.py               # default: husarion_world
ros2 launch sar_gazebo gazebo.launch.py world:=office # project office world
ros2 launch sar_gazebo gazebo.launch.py rviz:=True    # also open RViz
```

### Launch arguments

- `world` (default `husarion_world`) — preset name or absolute SDF path. Local
  presets: `office`. Anything else is resolved from `husarion_gz_worlds`
  (`husarion_world`, `husarion_office`, `sonoma_raceway`, `empty_with_plugins`).
- `x`, `y`, `z`, `yaw` — spawn pose overrides. Empty string picks the
  per-world default (see `WORLD_SPAWN_DEFAULTS` in the launch file).
- `rviz` (default `False`) — launch RViz with `config/rosbot.rviz`.

Default spawn poses live in `WORLD_SPAWN_DEFAULTS` inside `gazebo.launch.py`.
Husarion worlds use `(0, 0, 0.2, 0)`; `office` uses `(-10.0, 1.5, 0.3, 0)`. Any
axis can be overridden on the command line:

```bash
ros2 launch sar_gazebo gazebo.launch.py world:=office x:=-5 y:=0 z:=0.3 yaw:=1.57
```

### ArUco markers

`aruco1` is spawned automatically *only* when `world=husarion_world` — its pose
`(3, -2, 0.8)` is hand-placed for that map. For other worlds, add markers by
extending the launch or spawning them manually with `ros_gz_sim create`.

## Adding a new world

1. Drop the SDF under `worlds/<name>.sdf`. Use the plugin block from
   `worlds/office.sdf` as a starting template (physics, sensors, IMU, contact,
   scene broadcaster, user commands).
2. If the world references a custom model, add it under `models/<model_name>/`
   with a `model.config` and `model.sdf`; reference it with
   `model://<model_name>/...` from the world SDF.
3. Register the preset in `gazebo.launch.py`:
   - Add the name to `SAR_WORLDS` so the resolver picks the package-local path.
   - Optionally add an entry to `WORLD_SPAWN_DEFAULTS` for a sensible spawn pose.
   - If the world needs world-specific props (like `aruco1`), gate them the
     same way `spawn_aruco_setup` does.
4. Install the new files from `setup.py` (`worlds/*.sdf` is already globbed;
   new model directories must be listed explicitly).
5. Rebuild: `pixi run build-pkg sar_gazebo`.

## Notes on `office`

- Source mesh is a CAD STL export in millimetres with Y-axis up. The model SDF
  applies `<scale>0.001 0.001 0.001</scale>` and `roll = π/2` on the link pose
  to convert to Gazebo's metre / Z-up convention.
- The STL is reused as both visual and collision geometry. If physics lags on a
  larger map, swap in a simplified collision mesh.
- Extents after transformation: X ∈ [-21.2, 0.3] m, Y ∈ [-10.3, 13.3] m,
  Z ∈ [-0.3, 4.0] m (ceiling ~4 m).
