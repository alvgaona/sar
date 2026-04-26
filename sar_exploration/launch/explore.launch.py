from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sar_exploration_dir = FindPackageShare('sar_exploration')
    sar_slam_dir = FindPackageShare('sar_slam')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    explore_lite_launch_path = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    include_slam = LaunchConfiguration('include_slam')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([sar_exploration_dir, 'config', 'config.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    declare_include_slam_cmd = DeclareLaunchArgument(
        'include_slam',
        default_value='true',
        description='Include sar_slam. Set false when SLAM is composed at a higher level.',
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sar_slam_dir, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(include_slam),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            declare_include_slam_cmd,
            slam_launch,
            nav2_bringup_launch,
            explore_lite_launch,
        ]
    )
