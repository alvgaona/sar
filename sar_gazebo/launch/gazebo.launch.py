from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
import os

PACKAGE_NAME = "sar_gazebo"

SAR_WORLDS = {"office"}

WORLD_SPAWN_DEFAULTS = {
    "office": {"x": "-10.0", "y": "1.5", "z": "0.3", "yaw": "0.0"},
}
DEFAULT_SPAWN = {"x": "0.0", "y": "0.0", "z": "0.2", "yaw": "0.0"}

ARUCO_ENABLED_WORLDS = {"husarion_world"}

SUPPORTED_ROBOTS = {"rosbot", "sar"}


def resolve_world_name(context):
    return LaunchConfiguration("world").perform(context)


def resolve_robot(context):
    robot = LaunchConfiguration("robot").perform(context)
    if robot not in SUPPORTED_ROBOTS:
        raise ValueError(f"Unsupported robot '{robot}'. Choose one of {sorted(SUPPORTED_ROBOTS)}.")
    return robot


def resolve_world_path(context):
    world = resolve_world_name(context)
    if os.path.isabs(world) and os.path.isfile(world):
        return world
    if world in SAR_WORLDS:
        return os.path.join(get_package_share_directory(PACKAGE_NAME), "worlds", f"{world}.sdf")
    return os.path.join(get_package_share_directory("husarion_gz_worlds"), "worlds", f"{world}.sdf")


def resolve_spawn(context):
    world = resolve_world_name(context)
    defaults = WORLD_SPAWN_DEFAULTS.get(world, DEFAULT_SPAWN)
    return {
        axis: (LaunchConfiguration(axis).perform(context) or defaults[axis])
        for axis in ("x", "y", "z", "yaw")
    }


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value="husarion_world",
        description=(
            "World to load. Either a preset name (e.g. 'husarion_world', 'husarion_office', "
            "'sonoma_raceway', 'empty_with_plugins', 'office') or an absolute path to an SDF file."
        ),
    )

    declare_robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="rosbot",
        description="Which robot to spawn: 'rosbot' (Husarion ROSbot XL) or 'sar' (custom SAR platform).",
        choices=sorted(SUPPORTED_ROBOTS),
    )

    declare_x_arg = DeclareLaunchArgument("x", default_value="", description="Robot spawn X (override).")
    declare_y_arg = DeclareLaunchArgument("y", default_value="", description="Robot spawn Y (override).")
    declare_z_arg = DeclareLaunchArgument("z", default_value="", description="Robot spawn Z (override).")
    declare_yaw_arg = DeclareLaunchArgument("yaw", default_value="", description="Robot spawn yaw (override).")

    models_path = PathJoinSubstitution(
        [FindPackageShare(PACKAGE_NAME), "models"]
    )

    husarion_models_path = PathJoinSubstitution(
        [FindPackageShare("husarion_gz_worlds"), "models"]
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            os.pathsep,
            models_path,
            os.pathsep,
            husarion_models_path
        ]
    )

    def gz_sim_setup(context):
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "gz_log_level": "1",
                    "gz_world": resolve_world_path(context),
                }.items(),
            )
        ]

    gz_sim = OpaqueFunction(function=gz_sim_setup)

    def gz_bridge_setup(context):
        robot = resolve_robot(context)
        if robot == "sar":
            config = os.path.join(
                get_package_share_directory(PACKAGE_NAME), "config", "gz_bridge.yaml"
            )
        else:
            config = os.path.join(
                get_package_share_directory("rosbot_gazebo"), "config", "gz_bridge.yaml"
            )
        return [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_bridge",
                parameters=[{"config_file": config}],
            )
        ]

    gz_bridge = OpaqueFunction(function=gz_bridge_setup)

    def spawn_robot_setup(context):
        robot = resolve_robot(context)
        spawn = resolve_spawn(context)

        if robot == "rosbot":
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("rosbot_gazebo"),
                                "launch",
                                "spawn_robot.launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "robot_model": "rosbot_xl",
                        "configuration": "autonomy",
                        "x": spawn["x"],
                        "y": spawn["y"],
                        "z": spawn["z"],
                        "yaw": spawn["yaw"],
                    }.items(),
                )
            ]

        description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("sar_description"), "launch", "description.launch.py"]
                )
            ),
            launch_arguments={"use_sim": "true"}.items(),
        )

        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_sar",
            arguments=[
                "-name", "sar_robot",
                "-topic", "robot_description",
                "-x", spawn["x"],
                "-y", spawn["y"],
                "-z", spawn["z"],
                "-Y", spawn["yaw"],
            ],
            output="screen",
        )

        controllers_config = os.path.join(
            get_package_share_directory("sar_control"), "config", "controllers.yaml"
        )

        spawn_joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--param-file", controllers_config],
            output="screen",
        )

        spawn_drive_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mecanum_drive_controller", "--param-file", controllers_config],
            output="screen",
        )

        return [
            description_launch,
            spawn_entity,
            spawn_joint_state_broadcaster,
            spawn_drive_controller,
        ]

    spawn_robot = OpaqueFunction(function=spawn_robot_setup)

    def spawn_aruco_setup(context):
        if resolve_world_name(context) not in ARUCO_ENABLED_WORLDS:
            return []
        return [
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "aruco1",
                    "-file", os.path.join(
                        get_package_share_directory(PACKAGE_NAME), "models", "aruco1", "model.sdf"
                    ),
                    "-x", "3.0",
                    "-y", "-2.0",
                    "-z", "0.8",
                    "-R", "0.0",
                    "-P", "0.0",
                    "-Y", "1.59",
                ],
                output="screen",
            )
        ]

    spawn_aruco = OpaqueFunction(function=spawn_aruco_setup)

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(PACKAGE_NAME), "config", "rosbot.rviz"]
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        launch_arguments={
            "rviz_config": rviz_config,
            "use_sim": "True",
        }.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            declare_rviz_arg,
            declare_world_arg,
            declare_robot_arg,
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_yaw_arg,
            set_gazebo_model_path,
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            spawn_robot,
            spawn_aruco,
            rviz_launch,
        ]
    )
