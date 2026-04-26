# sar_bringup

Top-level launches that compose the SAR stack: `sim.launch.py` brings up
Gazebo + SLAM + exploration + perception; `real.launch.py` swaps Gazebo for
`sar_control` to run on the real robot. Both read a YAML config to decide
which stages run and what params each gets.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_bringup sim.launch.py
ros2 launch sar_bringup real.launch.py
ros2 launch sar_bringup sim.launch.py config_file:=/path/to/custom.yaml
```

## Notes

- The `config_file` argument points at `config/sim.yaml` or `config/real.yaml`
  by default. Each top-level key (`gazebo`, `control`, `slam`, `exploration`,
  `perception`) gates the corresponding `IncludeLaunchDescription` and forwards
  a small set of args (e.g. `params_file`, `marker_size`, `image_topic`).
- Set `<stage>.enabled: false` in the YAML to skip a stage without editing
  the launch.
- `slam`/`exploration` accept an empty `params_file` — when empty, each
  package's launch falls back to its own default config.
