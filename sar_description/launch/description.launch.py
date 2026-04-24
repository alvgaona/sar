from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_robot_description(context):
    use_sim = LaunchConfiguration("use_sim").perform(context)
    mock = LaunchConfiguration("mock").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    controllers_config = LaunchConfiguration("controllers_config").perform(context)

    if not controllers_config:
        yaml = "controllers_sim.yaml" if use_sim.lower() == "true" else "controllers.yaml"
        controllers_config = PathJoinSubstitution(
            [FindPackageShare("sar_control"), "config", yaml]
        ).perform(context)

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("sar_description"), "urdf", "sar_robot.urdf.xacro"]
            ),
            " use_sim:=",
            use_sim,
            " mock:=",
            mock,
            " namespace:=",
            namespace,
            " controllers_config:=",
            controllers_config,
        ]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_content},
                {"use_sim_time": LaunchConfiguration("use_sim")},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "controllers_config",
                default_value="",
                description=(
                    "Path to controllers yaml. If empty, selects controllers_sim.yaml "
                    "when use_sim:=true, else controllers.yaml."
                ),
            ),
            OpaqueFunction(function=_build_robot_description),
        ]
    )
