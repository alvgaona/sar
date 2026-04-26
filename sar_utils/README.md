# sar_utils

Teleop helpers: a `joy_node` + `teleop_twist_joy` launch for joystick
control, and a small wrapper for the standard keyboard teleop.

Workspace-level setup is in the [root README](../README.md).

## Run

```bash
ros2 launch sar_utils joystick.launch.py
ros2 launch sar_utils joystick.launch.py \
    cmd_vel_topic:=/mecanum_drive_controller/cmd_vel stamped:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=false --remap cmd_vel:=cmd_vel
```

## Notes

- `cmd_vel_topic` defaults to `/cmd_vel`. Use the controller's namespaced
  topic when going around `velocity_smoother`/Nav2.
- `stamped:=true` switches to `TwistStamped` (required by some controllers
  built against newer `ros2_controllers`).
- The joystick has an enable/turbo gate: hold a shoulder button for
  messages to be published.
- Joystick mappings, deadzone, and axes live in `config/joystick.yaml`.
