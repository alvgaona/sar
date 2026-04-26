# sar_slam

Wraps `slam_toolbox` in async mode for the SAR robot. Reads
`/scan_filtered` and publishes `map → odom`. The `odom → base_link` half of
the chain comes from the wheel-encoder odometry of the mecanum controller
(real robot or sim) — there's no EKF in the loop.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_slam slam.launch.py use_sim_time:=true
ros2 launch sar_slam slam.launch.py slam_params_file:=/path/to/custom.yaml
```

## Notes

- Defaults to `config/config.yaml`. Override the whole file with
  `slam_params_file:=...`.
- `/scan_filtered` is provided by the Gazebo launch (via `laser_filters`)
  or by the real robot's RPLIDAR pipeline. SLAM does not subscribe to
  `/scan` directly.
