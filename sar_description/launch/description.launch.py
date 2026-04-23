from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    mock = LaunchConfiguration("mock")
    namespace = LaunchConfiguration("namespace")

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
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("namespace", default_value=""),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description_content},
                    {"use_sim_time": LaunchConfiguration("use_sim")},
                ],
            ),
        ]
    )
