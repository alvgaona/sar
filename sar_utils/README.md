# sar_utils

## Description

The `sar_utils` package provides utility tools and configurations for the SAR (Search and Rescue) project. It includes launch files and configurations for joystick and keyboard teleoperation of ROS2 robots.

## Features

- **Joystick Teleoperation**: Launch file to control the robot using a joystick via the `joy` and `teleop_twist_joy` packages.
- **Keyboard Teleoperation**: Instructions and configurations for keyboard-based control.
- **Configurable Parameters**: YAML configuration files for customizing joystick mappings and settings.

## Dependencies (Nodes)

- `joy` package for joystick input
- `teleop_twist_joy` package for teleoperation

## Installation

1. Ensure you have ROS2 installed and sourced.
2. Clone or navigate to your ROS2 workspace (e.g., `sar_ws`).
3. Build the package using colcon:

   ```bash
   colcon build --packages-select sar_utils
   ```

4. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Usage

### Joystick Teleoperation

To launch the joystick teleoperation:

```bash
ros2 launch sar_utils joystick.launch.py cmd_vel_topic:=/mecanum_drive_controller/cmd_vel stamped:=true
```

- `cmd_vel_topic`: The topic to publish velocity commands (default: `/cmd_vel`)
- `stamped`: Whether to use `TwistStamped` instead of `Twist` (default: `false`)

This will start the `joy_node` and `teleop_node` with the configured parameters.

(You must press one of the shoulder buttons to enable normal or turbo mode for the joystick to publish messages).

### Keyboard Teleoperation

For keyboard control, run the following in a terminal with focus:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false --remap cmd_vel:=cmd_vel
```

Adjust the parameters as needed.

## Configuration

- **joystick.yaml**: Contains parameters for joystick device, deadzone, axes mappings, and button configurations.

You can modify `config/joystick.yaml` to suit your joystick setup.

## License

This package is licensed under the Apache-2.0 License. See the LICENSE file for details.
