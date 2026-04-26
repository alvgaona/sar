#!/usr/bin/env python3

import math
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sar_msgs.msg import ArucoMsg


class State(Enum):
    EXPLORING = auto()
    MARKER_FOUND = auto()
    RETURNING_HOME = auto()
    ARRIVED = auto()
    FAILED = auto()


def yaw_to_quat(yaw):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ExploreAndDetect(Node):
    def __init__(self):
        super().__init__('explore_and_detect')

        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('nav_action', 'navigate_to_pose')

        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_yaw = self.get_parameter('home_yaw').value
        self.map_frame = self.get_parameter('map_frame').value
        nav_action = self.get_parameter('nav_action').value

        self.state = State.EXPLORING

        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.aruco_sub = self.create_subscription(
            ArucoMsg, '/aruco/detection', self.aruco_callback, 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)

        self.get_logger().info(
            f'ExploreAndDetect started — home=({self.home_x:.2f}, '
            f'{self.home_y:.2f}, {self.home_yaw:.2f}) in {self.map_frame}'
        )

    def aruco_callback(self, msg):
        if self.state != State.EXPLORING or not msg.detected:
            return

        self.state = State.MARKER_FOUND
        self.get_logger().info(
            f'ArUco detected at {msg.distance:.2f} m. Pausing exploration.'
        )
        self.resume_pub.publish(Bool(data=False))
        # Give explore_lite time to receive the pause and stop scheduling new
        # goals. Without this, its planner_frequency timer can squeeze in one
        # more frontier goal that preempts our home goal on the shared
        # navigate_to_pose action server, surfacing as STATUS_CANCELED.
        self._return_timer = self.create_timer(0.5, self._return_home_once)

    def _return_home_once(self):
        self._return_timer.cancel()
        if self.state != State.MARKER_FOUND:
            return
        self.return_home()

    def return_home(self):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server "{self.nav_client._action_name}" not available; cannot return home.'
            )
            self.state = State.FAILED
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.home_x)
        pose.pose.position.y = float(self.home_y)
        qx, qy, qz, qw = yaw_to_quat(self.home_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        self.state = State.RETURNING_HOME
        self.get_logger().info('Sending NavigateToPose home goal.')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Home navigation goal rejected by Nav2.')
            self.state = State.FAILED
            return
        self.get_logger().info('Home goal accepted; en route.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        # status: 4 = SUCCEEDED in action_msgs/GoalStatus
        status = future.result().status
        if status == 4:
            self.state = State.ARRIVED
            self.get_logger().info('Arrived home. Mission complete.')
        else:
            self.state = State.FAILED
            self.get_logger().error(f'Return-to-home failed (status={status}).')


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
