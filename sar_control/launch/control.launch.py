from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration("mock")
    baud_rate = LaunchConfiguration("baud_rate")
    device = LaunchConfiguration("device")

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sar_control"),
                    "launch",
                    "joystick.launch.py",
                ]
            )
        ),
        launch_arguments={"cmd_vel_topic": "/mecanum_drive_controller/cmd_vel"}.items(),
        condition=LaunchConfigurationEquals("manual", "true"),
    )
    # TODO: Add a Node or LaunchDescription for non manual mode (e.g. autonomous navigation)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("sar_description"), "urdf", "sar_robot.urdf.xacro"]
            ),
            " mock:=",
            mock,
            " baud_rate:=",
            baud_rate,
            " device:=",
            device,
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    # Command(["ros2 param get --hide-type /robot_state_publisher robot_description"])

    controller_config = PathJoinSubstitution(
        [FindPackageShare("sar_control"), "config", "controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner, mecanum_drive_controller_spawner],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mock",
                default_value="false",
                description="Whether to use the mock hardware interface",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="115200",
                description="Baud rate for the serial connection",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/ttyACM0",
                description="Device path for the serial connection",
            ),
            DeclareLaunchArgument(
                "manual",
                default_value="false",
                description="Whether to use manual control (joystick)",
                choices=["true", "false"],
            ),
            controller_manager,
            robot_state_publisher,
            delayed_joint_state_broadcaster,
            joystick_launch,
        ]
    )
