from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    home_x = LaunchConfiguration('home_x')
    home_y = LaunchConfiguration('home_y')
    home_yaw = LaunchConfiguration('home_yaw')
    map_frame = LaunchConfiguration('map_frame')

    declare_home_x = DeclareLaunchArgument('home_x', default_value='0.0')
    declare_home_y = DeclareLaunchArgument('home_y', default_value='0.0')
    declare_home_yaw = DeclareLaunchArgument('home_yaw', default_value='0.0')
    declare_map_frame = DeclareLaunchArgument('map_frame', default_value='map')

    mission_node = Node(
        package='sar_mission',
        executable='explore_and_detect',
        name='explore_and_detect',
        output='screen',
        parameters=[{
            'home_x': home_x,
            'home_y': home_y,
            'home_yaw': home_yaw,
            'map_frame': map_frame,
        }],
    )

    return LaunchDescription([
        declare_home_x,
        declare_home_y,
        declare_home_yaw,
        declare_map_frame,
        mission_node,
    ])
