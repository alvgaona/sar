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

    slam_cfg = config.get("slam", {})
    exploration_cfg = config.get("exploration", {})
    perception_cfg = config.get("perception", {})
    gazebo_cfg = config.get("gazebo", {})
    mission_cfg = config.get("mission", {})

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_gazebo"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "rviz": str(gazebo_cfg.get("rviz", True)),
            "robot": gazebo_cfg.get("robot", "rosbot"),
            "world": gazebo_cfg.get("world", "office"),
        }.items(),
    )

    slam_args = {"use_sim_time": "true"}
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

    exploration_args = {
        "use_sim_time": "true",
        # SLAM is composed at the bringup level (or disabled here); never let
        # sar_exploration spin up a second slam_toolbox node.
        "include_slam": "false",
    }
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
            "use_realsense": "false",
            "marker_size": str(perception_cfg.get("marker_size", 0.75)),
            "aruco_dict": perception_cfg.get("aruco_dict", "DICT_6X6_1000"),
            "image_topic": perception_cfg.get("image_topic", "/oak/rgb/color"),
        }.items(),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sar_mission"), "launch", "explore_and_detect.launch.py"]
            )
        ),
        launch_arguments={
            "home_x": str(mission_cfg.get("home_x", 0.0)),
            "home_y": str(mission_cfg.get("home_y", 0.0)),
            "home_yaw": str(mission_cfg.get("home_yaw", 0.0)),
            "map_frame": mission_cfg.get("map_frame", "map"),
        }.items(),
    )

    actions = [gazebo]
    if slam_cfg.get("enabled", True):
        actions.append(slam)
    if exploration_cfg.get("enabled", True):
        actions.append(exploration)
    if perception_cfg.get("enabled", True):
        actions.append(perception)
    if mission_cfg.get("enabled", False):
        actions.append(mission)

    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("sar_bringup"), "config", "sim.yaml"]
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
