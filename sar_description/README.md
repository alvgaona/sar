# sar_description

URDF/xacro for the SAR robot plus a single launch (`description.launch.py`)
that runs `robot_state_publisher` with the URDF expanded for either the real
robot or the Gazebo sim.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_description description.launch.py                 # real
ros2 launch sar_description description.launch.py use_sim:=true   # sim
ros2 launch sar_description description.launch.py mock:=true
```

## Notes

- A single xacro (`urdf/sar_robot.urdf.xacro`) drives both real and sim. The
  `use_sim` arg switches:
  - wheel geometry (real: 30 mm radius, 200×190 mm separation; sim: ROSbot
    XL physics — 50 mm radius, 170×270 mm — chosen because ODE struggles
    with mecanum at the real robot's small scale),
  - body inertia/mass,
  - the `ros2_control` plugin (`sar_control/SarSystemHardware` vs
    `ign_ros2_control/IgnitionSystem`),
  - the wheel mesh (Husarion's `mecanum_a/b.dae` from `rosbot_description`
    in sim, internal mesh on the real robot),
  - inclusion of `gazebo.urdf.xacro` (sensors + ros2_control plugin).
- `controllers_config` defaults are auto-selected from `sar_control` based on
  `use_sim`. Pass an explicit path to override.
- The Gazebo sensor block defines an `ign_ros2_control-system` plugin (Fortress
  / gz-sim 6 — not the `gz_ros2_control` variant), a 360° gpu_lidar on
  `laser_frame`, an RGB camera on `camera_frame` (pointing along +X), and an
  IMU on `imu_link`.
- Per-wheel friction uses anisotropic ODE params with `fdir1` expressed in
  `base_link` to model mecanum roller chirality.
