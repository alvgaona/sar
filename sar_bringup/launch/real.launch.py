import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    config_file = LaunchConfiguration("config_file").perform(context)

    with open(config_file) as f:
        config = yaml.safe_load(f)

    control_cfg = config.get("control", {})
    slam_cfg = config.get("slam", {})
    exploration_cfg = config.get("exploration", {})
    perception_cfg = config.get("perception", {})

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_control"), "launch", "control.launch.py"]
            )
        ),
        launch_arguments={
            "mock": str(control_cfg.get("mock", False)).lower(),
        }.items(),
    )

    slam_args = {"use_sim_time": "false"}
    if slam_cfg.get("params_file"):
        slam_args["slam_params_file"] = slam_cfg["params_file"]
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_slam"), "launch", "slam.launch.py"]
            )
        ),
        launch_arguments=slam_args.items(),
    )

    exploration_args = {"use_sim_time": "false"}
    if exploration_cfg.get("params_file"):
        exploration_args["params_file"] = exploration_cfg["params_file"]
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_exploration"), "launch", "explore.launch.py"]
            )
        ),
        launch_arguments=exploration_args.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_perception"), "launch", "perception.launch.py"]
            )
        ),
        launch_arguments={
            "use_realsense": str(perception_cfg.get("use_realsense", True)).lower(),
            "marker_size": str(perception_cfg.get("marker_size", 0.75)),
            "aruco_dict": perception_cfg.get("aruco_dict", "DICT_6X6_1000"),
            "image_topic": perception_cfg.get("image_topic", "/camera/color/image_raw"),
        }.items(),
    )

    actions = [control]
    if slam_cfg.get("enabled", True):
        actions.append(slam)
    if exploration_cfg.get("enabled", True):
        actions.append(exploration)
    if perception_cfg.get("enabled", True):
        actions.append(perception)

    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("sar_bringup"), "config", "real.yaml"]
    )

    declare_config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the bringup config YAML file",
    )

    return LaunchDescription([
        declare_config_arg,
        OpaqueFunction(function=launch_setup),
    ])
