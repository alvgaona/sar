# sar_gazebo

Gazebo (`gz-sim`) assets and the launch glue that brings up either a Husarion
stock world or a project-local world, with either the Husarion ROSbot XL or
the custom SAR robot, plus the bridge and a preconfigured RViz.

Workspace-level setup (clone, install, build) is in the
[root README](../README.md). For the exact arg defaults, spawn poses, and
plugin lists, read `launch/gazebo.launch.py` and the SDFs.

## Run

```bash
ros2 launch sar_gazebo gazebo.launch.py                    # office, rosbot
ros2 launch sar_gazebo gazebo.launch.py robot:=sar
ros2 launch sar_gazebo gazebo.launch.py world:=husarion_world
ros2 launch sar_gazebo gazebo.launch.py rviz:=True
```

`world` accepts a preset name or an absolute SDF path. Unknown names are
resolved under `husarion_gz_worlds/worlds/`. `x y z yaw` override the
per-world spawn defaults.

## Notes

- `GZ_SIM_RESOURCE_PATH` is extended at launch with `sar_gazebo/models/`,
  `husarion_gz_worlds/models/`, and the `share/` parents of `sar_description`
  and `rosbot_description` — required so `model://<pkg>/...` URIs emitted by
  `ros_gz_sim` (when converting URDF `package://` references) resolve.
- For `robot:=sar` the entity spawn and controller spawners are staggered
  with `TimerAction` to avoid races between `ros_gz_sim create -topic
  robot_description` and `robot_state_publisher` publishing it.
- `laser_filters/scan_to_scan_filter_chain` runs alongside `sar` to produce
  `/scan_filtered` from `/scan`, matching what the SLAM and Nav2 stacks
  expect.
- `arucobox` carries one ArUco marker on every face (texture `aruco1.png`
  reused via UVs). It auto-spawns in worlds listed in `ARUCO_BOX_POSES`.
- The `office` STL is authored Z-up with forward = +Y and is reused for both
  visual and collision; if physics lags, swap in a simplified collision mesh.

## Adding a new world

1. Drop the SDF under `worlds/<name>.sdf` (use `office.sdf` as a plugin-block
   template).
2. If it references a custom model, add `models/<model_name>/` with
   `model.config` + `model.sdf`.
3. In `gazebo.launch.py`, add the name to `SAR_WORLDS`, optionally to
   `WORLD_SPAWN_DEFAULTS`, and to `ARUCO_BOX_POSES` if you want the marker
   box.
4. Install new model directories from `setup.py` (worlds glob is automatic).
