from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

PACKAGE_NAME = "sar_perception"

def generate_launch_description():
    # Launch configuration
    marker_size = LaunchConfiguration("marker_size")
    aruco_dict = LaunchConfiguration("aruco_dict")
    image_topic = LaunchConfiguration("image_topic")
    camera_fx = LaunchConfiguration("camera_fx")
    camera_fy = LaunchConfiguration("camera_fy")
    camera_cx = LaunchConfiguration("camera_cx")
    camera_cy = LaunchConfiguration("camera_cy")

    # Declare launch arguments
    declare_marker_size_arg = DeclareLaunchArgument(
        "marker_size", default_value="0.75", description="ArUco marker size in meters"
    )

    declare_aruco_dict_arg = DeclareLaunchArgument(
        "aruco_dict", default_value="DICT_6X6_1000", description="ArUco dictionary to use"
    )

    declare_image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/oak/rgb/color", description="Camera image topic"
    )

    declare_camera_fx_arg = DeclareLaunchArgument(
        "camera_fx", default_value="1108.51", description="Camera focal length x"
    )

    declare_camera_fy_arg = DeclareLaunchArgument(
        "camera_fy", default_value="1108.51", description="Camera focal length y"
    )

    declare_camera_cx_arg = DeclareLaunchArgument(
        "camera_cx", default_value="640.0", description="Camera principal point x"
    )

    declare_camera_cy_arg = DeclareLaunchArgument(
        "camera_cy", default_value="360.0", description="Camera principal point y"
    )

    # ArUco detection node
    aruco_detector = Node(
        package=PACKAGE_NAME,
        executable="aruco_detector",
        name="aruco_detector",
        parameters=[
            {"marker_size": marker_size},          # ArUco marker size
            {"aruco_dict": aruco_dict},            # ArUco dictionary
            {"camera_fx": camera_fx},              # Camera focal length x
            {"camera_fy": camera_fy},              # Camera focal length y
            {"camera_cx": camera_cx},              # Camera principal point x
            {"camera_cy": camera_cy},              # Camera principal point y
            {"image_topic": image_topic},          # Camera image topic
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_marker_size_arg,
            declare_aruco_dict_arg,
            declare_image_topic_arg,
            declare_camera_fx_arg,
            declare_camera_fy_arg,
            declare_camera_cx_arg,
            declare_camera_cy_arg,
            aruco_detector,
        ]
    )
