#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sar_msgs.msg import ArucoMsg
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.30)
        self.declare_parameter('aruco_dict', 'DICT_6X6_1000')
        self.declare_parameter('image_topic', '/oak/rgb/color')
        self.declare_parameter('camera_info_topic', '')

        self.marker_size = self.get_parameter('marker_size').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        if not camera_info_topic:
            camera_info_topic = self.image_topic.rsplit('/', 1)[0] + '/camera_info'

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

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.aruco_pub = self.create_publisher(ArucoMsg, '/aruco/detection', 10)
        self.image_pub = self.create_publisher(Image, '/aruco/image', 10)

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

    def estimate_distance(self, corners):
        """
        Estimate distance to ArUco marker using perspective-n-point

        Args:
            corners: ArUco marker corners in image coordinates

        Returns:
            Distance to marker in meters
        """
        # Define 3D points of the marker in marker coordinate system
        # Marker is assumed to be a square with given size
        half_size = self.marker_size / 2.0
        object_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Image points are the detected corners
        image_points = corners.reshape((4, 2)).astype(np.float32)

        # Solve PnP to get rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs
        )

        if success:
            # Distance is the magnitude of the translation vector
            distance = np.linalg.norm(tvec)
            return distance
        else:
            return None

    def image_callback(self, msg):
        """Process camera image and detect ArUco markers"""
        if self.camera_matrix is None:
            self.get_logger().warn(
                'Waiting for camera_info before processing images',
                throttle_duration_sec=5.0,
            )
            return
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale for detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Ensure contiguous array (important for some OpenCV versions)
            gray = np.ascontiguousarray(gray)

            # Detect ArUco markers (Legacy API)
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, 
                self.aruco_dict, 
                parameters=self.aruco_params
            )

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                distance = self.estimate_distance(corners[0])

                if distance is not None:
                    label = f'ID:{ids[0][0]} {distance:.2f}m'
                    org = (int(corners[0][0][0][0]), int(corners[0][0][0][1]) - 10)
                    cv2.putText(cv_image, label, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    detection_msg = ArucoMsg()
                    detection_msg.detected = True
                    detection_msg.distance = float(distance)
                    self.aruco_pub.publish(detection_msg)
                    self.get_logger().debug(
                        f'ArUco marker {ids[0][0]} detected at {distance:.3f}m'
                    )
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
