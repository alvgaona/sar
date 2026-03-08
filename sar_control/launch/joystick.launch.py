from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cmd_vel = LaunchConfiguration("cmd_vel_topic")
    joy_config = PathJoinSubstitution(
        [FindPackageShare("sar_control"), "config", "joystick.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_config],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_config],
        remappings=[("/cmd_vel", cmd_vel)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Topic for the command velocity",
            ),
            joy_node,
            teleop_node,
        ]
    )
