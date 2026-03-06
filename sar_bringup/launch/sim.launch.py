from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_gazebo"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"rviz": rviz}.items(),
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_exploration"), "launch", "explore.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_perception"), "launch", "perception.launch.py"]
            )
        ),
        launch_arguments={"use_realsense": "false"}.items(),
    )

    return LaunchDescription([
        declare_rviz_arg,
        gazebo,
        exploration,
        perception,
    ])
