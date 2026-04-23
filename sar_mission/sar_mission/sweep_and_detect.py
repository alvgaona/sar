#!/usr/bin/env python3

import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin

from sar_mission.marker_monitor import MarkerMonitor


class State(Enum):
    WAITING_FOR_MAP = auto()
    SWEEPING = auto()
    ROTATING = auto()
    FINISHED = auto()


class SweepAndDetect(Node):
    def __init__(self):
        super().__init__('sweep_and_detect')

        self.declare_parameter('confirmation_duration', 1.0)
        self.declare_parameter('sweep_spacing', 2.0)

        self.confirmation_duration = self.get_parameter('confirmation_duration').value
        self.sweep_spacing = self.get_parameter('sweep_spacing').value

        self.state = State.WAITING_FOR_MAP
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.marker_found = False

        self.result_pub = self.create_publisher(String, '/mission/result', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.current_goal_handle = None

        self.monitor = MarkerMonitor(
            self, self.confirmation_duration, self.on_marker_confirmed
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 1
        )

        self.get_logger().info(
            f'SweepAndDetect mission started '
            f'(sweep spacing: {self.sweep_spacing}m)'
        )

    def on_marker_confirmed(self, distance):
        if self.state == State.FINISHED:
            return

        self.marker_found = True
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.state = State.FINISHED

        result = String()
        result.data = 'marker_found'
        self.result_pub.publish(result)
        self.get_logger().info('Mission complete: marker found during sweep.')

    def map_callback(self, msg):
        if self.state != State.WAITING_FOR_MAP:
            return

        self.get_logger().info('Map received, generating waypoints.')
        self.waypoints = self.generate_waypoints(msg)

        if not self.waypoints:
            self.get_logger().warn('No valid waypoints generated. Mission failed.')
            self.finish('no_waypoints')
            return

        self.state = State.SWEEPING
        self.current_waypoint_idx = 0
        self.navigate_to_waypoint()

    def generate_waypoints(self, grid):
        info = grid.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height
        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        step = max(1, int(self.sweep_spacing / resolution))
        waypoints = []

        for row in range(step // 2, height, step):
            for col in range(step // 2, width, step):
                if data[row, col] != 0:
                    continue

                r_min = max(0, row - step // 4)
                r_max = min(height, row + step // 4)
                c_min = max(0, col - step // 4)
                c_max = min(width, col + step // 4)
                region = data[r_min:r_max, c_min:c_max]
                if np.any(region == 100) or np.any(region == -1):
                    continue

                wx = origin_x + (col + 0.5) * resolution
                wy = origin_y + (row + 0.5) * resolution
                waypoints.append((wx, wy))

        self.get_logger().info(f'Generated {len(waypoints)} sweep waypoints')
        return waypoints

    def navigate_to_waypoint(self):
        if self.marker_found or self.state == State.FINISHED:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().warn('Sweep complete. Marker not found.')
            self.finish('sweep_complete')
            return

        wx, wy = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/'
            f'{len(self.waypoints)}: ({wx:.1f}, {wy:.1f})'
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_accepted)

    def nav_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint rejected, skipping.')
            self.current_waypoint_idx += 1
            self.navigate_to_waypoint()
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result)

    def nav_result(self, future):
        self.current_goal_handle = None
        if self.marker_found or self.state == State.FINISHED:
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached. Spinning 360°.')
            self.start_spin()
        else:
            self.get_logger().warn(f'Navigation failed (status {status}), skipping waypoint.')
            self.current_waypoint_idx += 1
            self.navigate_to_waypoint()

    def start_spin(self):
        self.state = State.ROTATING

        goal = Spin.Goal()
        goal.target_yaw = 5.5
        goal.time_allowance = Duration(sec=30)

        self.get_logger().info(
            f'Requesting spin: target_yaw={goal.target_yaw:.2f} rad, '
            f'time_allowance={goal.time_allowance.sec}s'
        )

        self.spin_client.wait_for_server()
        future = self.spin_client.send_goal_async(
            goal, feedback_callback=self.spin_feedback
        )
        future.add_done_callback(self.spin_goal_accepted)

    def spin_feedback(self, feedback_msg):
        traveled = feedback_msg.feedback.angular_distance_traveled
        self.get_logger().info(f'Spin progress: {traveled:.2f} rad / 5.50 rad')

    def spin_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin rejected, moving to next waypoint.')
            self.current_waypoint_idx += 1
            self.state = State.SWEEPING
            self.navigate_to_waypoint()
            return

        self.get_logger().info('Spin goal accepted.')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result)

    def spin_result(self, future):
        self.current_goal_handle = None
        if self.marker_found or self.state == State.FINISHED:
            return

        status = future.result().status
        self.get_logger().info(f'Spin completed with status: {status}')

        self.state = State.SWEEPING
        self.current_waypoint_idx += 1
        self.navigate_to_waypoint()

    def finish(self, result_str):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.state = State.FINISHED
        result = String()
        result.data = result_str
        self.result_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = SweepAndDetect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
