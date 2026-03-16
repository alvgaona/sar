from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

PACKAGE_NAME = "sar_perception"

def generate_launch_description():
    use_realsense = LaunchConfiguration("use_realsense")
    marker_size = LaunchConfiguration("marker_size")
    aruco_dict = LaunchConfiguration("aruco_dict")
    image_topic = LaunchConfiguration("image_topic")
    camera_fx = LaunchConfiguration("camera_fx")
    camera_fy = LaunchConfiguration("camera_fy")
    camera_cx = LaunchConfiguration("camera_cx")
    camera_cy = LaunchConfiguration("camera_cy")
    rs_rgb_width = LaunchConfiguration("rs_rgb_width")
    rs_rgb_height = LaunchConfiguration("rs_rgb_height")
    rs_rgb_fps = LaunchConfiguration("rs_rgb_fps")

    declare_use_realsense_arg = DeclareLaunchArgument(
        "use_realsense", default_value="false", description="Launch RealSense D435i camera driver"
    )

    declare_marker_size_arg = DeclareLaunchArgument(
        "marker_size", default_value="0.30", description="ArUco marker size in meters"
    )

    declare_aruco_dict_arg = DeclareLaunchArgument(
        "aruco_dict", default_value="DICT_6X6_1000", description="ArUco dictionary to use"
    )

    declare_image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value=PythonExpression([
            "'/camera/color/image_raw' if '", use_realsense, "' == 'true' else '/oak/rgb/color'"
        ]),
        description="Camera image topic",
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

    declare_rs_rgb_width_arg = DeclareLaunchArgument(
        "rs_rgb_width", default_value="1280", description="RealSense RGB stream width"
    )

    declare_rs_rgb_height_arg = DeclareLaunchArgument(
        "rs_rgb_height", default_value="720", description="RealSense RGB stream height"
    )

    declare_rs_rgb_fps_arg = DeclareLaunchArgument(
        "rs_rgb_fps", default_value="30", description="RealSense RGB stream FPS"
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        parameters=[{
            "enable_color": True,
            "enable_depth": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "rgb_camera.color_profile": [
                rs_rgb_width, TextSubstitution(text="x"),
                rs_rgb_height, TextSubstitution(text="x"),
                rs_rgb_fps,
            ],
        }],
        output="screen",
        condition=IfCondition(use_realsense),
    )

    aruco_detector = Node(
        package=PACKAGE_NAME,
        executable="aruco_detector",
        name="aruco_detector",
        parameters=[
            {"marker_size": marker_size},
            {"aruco_dict": aruco_dict},
            {"camera_fx": camera_fx},
            {"camera_fy": camera_fy},
            {"camera_cx": camera_cx},
            {"camera_cy": camera_cy},
            {"image_topic": image_topic},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_realsense_arg,
            declare_marker_size_arg,
            declare_aruco_dict_arg,
            declare_image_topic_arg,
            declare_camera_fx_arg,
            declare_camera_fy_arg,
            declare_camera_cx_arg,
            declare_camera_cy_arg,
            declare_rs_rgb_width_arg,
            declare_rs_rgb_height_arg,
            declare_rs_rgb_fps_arg,
            realsense_node,
            aruco_detector,
        ]
    )
