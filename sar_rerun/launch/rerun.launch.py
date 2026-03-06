from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    map_topic = LaunchConfiguration("map_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/oak/rgb/color"),
        DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="sar_rerun",
            executable="rerun_bridge",
            name="rerun_bridge",
            parameters=[
                {"image_topic": image_topic},
                {"scan_topic": scan_topic},
                {"map_topic": map_topic},
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
    ])
