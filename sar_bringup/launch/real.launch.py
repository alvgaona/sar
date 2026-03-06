from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration("mock")

    declare_mock_arg = DeclareLaunchArgument(
        "mock", default_value="false", description="Run hardware in mock mode"
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_control"), "launch", "control.launch.py"]
            )
        ),
        launch_arguments={"mock": mock}.items(),
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_exploration"), "launch", "explore.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_perception"), "launch", "perception.launch.py"]
            )
        ),
        launch_arguments={"use_realsense": "true"}.items(),
    )

    return LaunchDescription([
        declare_mock_arg,
        control,
        exploration,
        perception,
    ])
