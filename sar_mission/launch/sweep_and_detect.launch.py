from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sar_mission',
            executable='sweep_and_detect',
            name='sweep_and_detect',
            output='screen',
        ),
    ])
