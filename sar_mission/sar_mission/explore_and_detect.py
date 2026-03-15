#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sar_msgs.msg import ArucoMsg


class State(Enum):
    EXPLORING = auto()
    MARKER_FOUND = auto()
    FINISHED = auto()


class ExploreAndDetect(Node):
    def __init__(self):
        super().__init__('explore_and_detect')

        self.state = State.EXPLORING

        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)

        self.aruco_sub = self.create_subscription(
            ArucoMsg,
            '/aruco/detection',
            self.aruco_callback,
            10
        )

        self.get_logger().info('ExploreAndDetect mission started: monitoring for ArUco markers')

    def aruco_callback(self, msg):
        if self.state != State.EXPLORING:
            return

        if not msg.detected:
            return

        self.state = State.MARKER_FOUND
        self.get_logger().info(
            f'ArUco marker detected at {msg.distance:.2f}m! Stopping exploration.'
        )

        stop_msg = Bool()
        stop_msg.data = False
        self.resume_pub.publish(stop_msg)

        self.state = State.FINISHED
        self.get_logger().info('Mission complete.')


def main(args=None):
    rclpy.init(args=args)
    node = ExploreAndDetect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
