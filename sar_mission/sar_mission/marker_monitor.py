#!/usr/bin/env python3

from rclpy.node import Node
from sar_msgs.msg import ArucoMsg


class MarkerMonitor:
    def __init__(self, node: Node, confirmation_duration: float, on_confirmed):
        self._node = node
        self._confirmation_duration = confirmation_duration
        self._on_confirmed = on_confirmed
        self._first_detection_time = None
        self._confirming = False

        self._sub = node.create_subscription(
            ArucoMsg, '/aruco/detection', self._aruco_callback, 10
        )

    @property
    def confirming(self):
        return self._confirming

    def reset(self):
        self._confirming = False
        self._first_detection_time = None

    def _aruco_callback(self, msg):
        now = self._node.get_clock().now()

        if not msg.detected:
            if self._confirming:
                self._node.get_logger().info('Detection lost, resetting confirmation.')
                self.reset()
            return

        if not self._confirming:
            self._confirming = True
            self._first_detection_time = now
            self._node.get_logger().info(
                f'ArUco marker detected at {msg.distance:.2f}m, confirming...'
            )
            return

        elapsed = (now - self._first_detection_time).nanoseconds / 1e9
        if elapsed >= self._confirmation_duration:
            self._node.get_logger().info(
                f'ArUco marker confirmed at {msg.distance:.2f}m after {elapsed:.1f}s.'
            )
            self._on_confirmed(msg.distance)
