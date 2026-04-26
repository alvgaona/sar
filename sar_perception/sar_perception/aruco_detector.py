#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sar_msgs.msg import ArucoMsg
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
import cv2
import numpy as np


def ema_pose(prev_xyz, prev_quat, new_xyz, new_quat, alpha):
    """EMA on position; weighted-average + renormalize on quaternion (with
    sign flip so antipodal quaternions are blended on the short arc)."""
    xyz = (1.0 - alpha) * prev_xyz + alpha * new_xyz
    if float(np.dot(prev_quat, new_quat)) < 0.0:
        new_quat = -new_quat
    quat = (1.0 - alpha) * prev_quat + alpha * new_quat
    n = float(np.linalg.norm(quat))
    if n > 0.0:
        quat = quat / n
    return xyz, quat


def rvec_tvec_to_pose(rvec, tvec):
    rot, _ = cv2.Rodrigues(rvec)
    qw = np.sqrt(max(0.0, 1.0 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2.0
    qx = (rot[2, 1] - rot[1, 2]) / (4.0 * qw) if qw > 1e-8 else 0.0
    qy = (rot[0, 2] - rot[2, 0]) / (4.0 * qw) if qw > 1e-8 else 0.0
    qz = (rot[1, 0] - rot[0, 1]) / (4.0 * qw) if qw > 1e-8 else 0.0
    pose = PoseStamped().pose
    pose.position.x = float(tvec[0])
    pose.position.y = float(tvec[1])
    pose.position.z = float(tvec[2])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.30)
        self.declare_parameter('aruco_dict', 'DICT_6X6_1000')
        self.declare_parameter('image_topic', '/oak/rgb/color')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('map_frame', 'map')
        # Mesh resource for the RViz marker. Empty → fall back to a green CUBE
        # the size of the ArUco marker. Use a package:// URI for ROS resources.
        self.declare_parameter('marker_mesh_resource', '')
        # Offset (meters) applied to the published pose along the marker's
        # local Z axis. The detected pose is at the marker's face center;
        # for a marker on a cube of side `marker_size`, set this to
        # -marker_size/2 to move the rendered mesh to the cube center.
        self.declare_parameter('marker_pose_offset_z', 0.0)
        # Per-marker EMA over poses in the map frame. 0 disables smoothing
        # (every detection is published as-is); 1 keeps only the first
        # detection. 0.1 typically converges in a few seconds.
        self.declare_parameter('smoothing_alpha', 0.1)

        self.marker_size = self.get_parameter('marker_size').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        if not camera_info_topic:
            camera_info_topic = self.image_topic.rsplit('/', 1)[0] + '/camera_info'
        self.map_frame = self.get_parameter('map_frame').value
        self.marker_mesh_resource = self.get_parameter('marker_mesh_resource').value
        self.marker_pose_offset_z = float(self.get_parameter('marker_pose_offset_z').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        # marker_id -> (np.array(xyz), np.array(quat xyzw))
        self.smoothed_pose = {}

        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco possible dictionaries
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        }

        if aruco_dict_name not in aruco_dict_map:
            self.get_logger().error(f'Unknown ArUco dictionary: {aruco_dict_name}')
            aruco_dict_name = 'DICT_6X6_1000'

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[aruco_dict_name])

        # Important: use DetectorParameters_create for Legacy API compatibility.
        # Otherwise, there's a segmentation fault
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.maxMarkerPerimeterRate = 10.0
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        self.aruco_params.adaptiveThreshWinSizeStep = 5
        # Tighter than the default 0.05 — sim-rendered markers produce a near-duplicate
        # candidate from anti-aliasing on the texture border, which the legacy detector
        # would otherwise reject as "too close to another marker".
        self.aruco_params.minMarkerDistanceRate = 0.01

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )

        self.aruco_pub = self.create_publisher(ArucoMsg, '/aruco/detection', 10)
        self.image_pub = self.create_publisher(Image, '/aruco/image', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco/marker', 10)

        self.get_logger().info(
            f'ArUco detector initialized - Dictionary: {aruco_dict_name}, '
            f'Marker size: {self.marker_size}m, '
            f'image: {self.image_topic}, camera_info: {camera_info_topic}'
        )

    def camera_info_callback(self, msg):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        d = list(msg.d) if msg.d else [0.0] * 5
        self.dist_coeffs = np.array(d, dtype=np.float32).reshape(-1, 1)
        self.get_logger().info(
            f'Camera intrinsics received: fx={self.camera_matrix[0,0]:.2f}, '
            f'fy={self.camera_matrix[1,1]:.2f}, cx={self.camera_matrix[0,2]:.2f}, '
            f'cy={self.camera_matrix[1,2]:.2f}'
        )

    def estimate_pose(self, corners):
        """Solve PnP for one marker. Returns (rvec, tvec, distance) or (None, None, None)."""
        half_size = self.marker_size / 2.0
        object_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0],
        ], dtype=np.float32)
        image_points = corners.reshape((4, 2)).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coeffs
        )
        if not success:
            return None, None, None
        return rvec, tvec, float(np.linalg.norm(tvec))

    def publish_map_marker(self, marker_id, rvec, tvec, header):
        # Apply the in-marker-frame Z offset (e.g. push the published pose from
        # the detected face to the cube center for a marker on a box).
        if self.marker_pose_offset_z != 0.0:
            R, _ = cv2.Rodrigues(rvec)
            offset_in_camera = R @ np.array(
                [0.0, 0.0, self.marker_pose_offset_z], dtype=np.float32
            )
            tvec = tvec.flatten() + offset_in_camera
        pose_in_camera = rvec_tvec_to_pose(rvec, tvec)

        # Use "latest available" rather than header.stamp: slam_toolbox publishes
        # map -> odom at its own rate (slower than the camera), so a stamped
        # lookup frequently fails with "extrapolation into the future". For a
        # visualization marker, latest-available is the standard tradeoff.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {self.map_frame} <- {header.frame_id} unavailable: {e}',
                throttle_duration_sec=5.0,
            )
            return

        pose_in_map = do_transform_pose(pose_in_camera, tf)

        # Per-marker EMA so the rendered mesh converges instead of jittering
        # on every single-frame PnP estimate.
        new_xyz = np.array([pose_in_map.position.x,
                            pose_in_map.position.y,
                            pose_in_map.position.z], dtype=np.float64)
        new_quat = np.array([pose_in_map.orientation.x,
                             pose_in_map.orientation.y,
                             pose_in_map.orientation.z,
                             pose_in_map.orientation.w], dtype=np.float64)
        if self.smoothing_alpha <= 0.0 or marker_id not in self.smoothed_pose:
            self.smoothed_pose[int(marker_id)] = (new_xyz, new_quat)
        else:
            prev_xyz, prev_quat = self.smoothed_pose[int(marker_id)]
            self.smoothed_pose[int(marker_id)] = ema_pose(
                prev_xyz, prev_quat, new_xyz, new_quat, self.smoothing_alpha
            )
        smooth_xyz, smooth_quat = self.smoothed_pose[int(marker_id)]
        pose_in_map.position.x = float(smooth_xyz[0])
        pose_in_map.position.y = float(smooth_xyz[1])
        pose_in_map.position.z = float(smooth_xyz[2])
        pose_in_map.orientation.x = float(smooth_quat[0])
        pose_in_map.orientation.y = float(smooth_quat[1])
        pose_in_map.orientation.z = float(smooth_quat[2])
        pose_in_map.orientation.w = float(smooth_quat[3])

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aruco'
        m.id = int(marker_id)
        m.action = Marker.ADD
        m.pose = pose_in_map
        m.frame_locked = False

        if self.marker_mesh_resource:
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = self.marker_mesh_resource
            m.mesh_use_embedded_materials = True
            # Mesh authored at unit scale → render 1:1.
            m.scale.x = m.scale.y = m.scale.z = 1.0
            # color.a=0 with embedded materials = use the texture as-is.
            m.color.a = 0.0
        else:
            m.type = Marker.CUBE
            m.scale.x = m.scale.y = m.scale.z = float(self.marker_size)
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8

        # lifetime=0 → persists; latest detection overwrites the same id+ns
        self.marker_pub.publish(m)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn(
                'Waiting for camera_info before processing images',
                throttle_duration_sec=5.0,
            )
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = np.ascontiguousarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY))

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                rvec, tvec, distance = self.estimate_pose(corners[0])

                if distance is not None:
                    label = f'ID:{ids[0][0]} {distance:.2f}m'
                    org = (int(corners[0][0][0][0]), int(corners[0][0][0][1]) - 10)
                    cv2.putText(cv_image, label, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    detection_msg = ArucoMsg()
                    detection_msg.detected = True
                    detection_msg.distance = float(distance)
                    self.aruco_pub.publish(detection_msg)

                    self.publish_map_marker(ids[0][0], rvec, tvec, msg.header)
                else:
                    self.get_logger().warn('Failed to estimate distance')
            else:
                detection_msg = ArucoMsg()
                detection_msg.detected = False
                detection_msg.distance = 0.0
                self.aruco_pub.publish(detection_msg)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
