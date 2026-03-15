from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_node = Node(
        package='sar_mission',
        executable='explore_and_detect',
        name='explore_and_detect',
        output='screen',
    )

    return LaunchDescription([
        mission_node,
    ])
