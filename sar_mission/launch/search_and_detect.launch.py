from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sar_mission',
            executable='search_and_detect',
            name='search_and_detect',
            output='screen',
        ),
    ])
