# sar_exploration

Frontier-based exploration: composes `sar_slam`, Nav2 (`nav2_bringup`'s
`navigation_launch.py`), and `explore_lite` from the `m-explore-ros2`
submodule. All three read from a single `config/config.yaml`.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_exploration explore.launch.py use_sim_time:=true
ros2 launch sar_exploration explore.launch.py params_file:=/path/to/custom.yaml
```

## Notes

- Same `params_file` is forwarded to Nav2 and read by `explore_lite` — the
  config groups are namespaced by node name (`explore_node`,
  `bt_navigator`, `controller_server`, `local_costmap`, `global_costmap`,
  `planner_server`, `behavior_server`, `velocity_smoother`, ...).
- `explore_node.max_frontier_distance` (in meters) caps how far frontiers
  can be from the robot — provided by our local patch to `m-explore-ros2`.
  Set to `0.0` to disable the cap.
- The Nav2 stack is configured for differential-drive motion: DWB samples
  no `vy`, accel/vel limits on y are zero, and the velocity smoother also
  zeroes y. This matches the controller-side limits in
  `sar_control/config/controllers_sim.yaml`.
- Costmap obstacle layers consume `/scan_filtered` (produced by
  `sar_gazebo`'s laser filter or by the real robot's RPLIDAR pipeline).
