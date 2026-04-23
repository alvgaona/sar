#!/usr/bin/env python3

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import ColorRGBA, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

from sar_mission.marker_monitor import MarkerMonitor
from sar_mission.coverage_planner import (
    generate_wall_following_waypoints,
    generate_lawnmower_waypoints,
    mark_covered,
    occupancy_to_numpy,
)


class State(Enum):
    WAITING_FOR_MAP = auto()
    WALL_FOLLOWING = auto()
    LAWNMOWER = auto()
    PAUSING = auto()
    FINISHED = auto()


class SearchAndDetect(Node):
    def __init__(self):
        super().__init__('search_and_detect')

        self.declare_parameter('confirmation_duration', 1.0)
        self.declare_parameter('wall_standoff', 1.0)
        self.declare_parameter('wall_spacing', 1.0)
        self.declare_parameter('sweep_spacing', 3.0)
        self.declare_parameter('min_clearance', 0.3)
        self.declare_parameter('sensor_range', 5.0)
        self.declare_parameter('pause_duration', 0.5)

        self.confirmation_duration = self.get_parameter('confirmation_duration').value
        self.wall_standoff = self.get_parameter('wall_standoff').value
        self.wall_spacing = self.get_parameter('wall_spacing').value
        self.sweep_spacing = self.get_parameter('sweep_spacing').value
        self.min_clearance = self.get_parameter('min_clearance').value
        self.sensor_range = self.get_parameter('sensor_range').value
        self.pause_duration = self.get_parameter('pause_duration').value

        self.state = State.WAITING_FOR_MAP
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.marker_found = False
        self.current_goal_handle = None
        self.pause_timer = None
        self.grid = None

        self.result_pub = self.create_publisher(String, '/mission/result', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/search/waypoints', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.monitor = MarkerMonitor(
            self, self.confirmation_duration, self.on_marker_confirmed
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 1
        )

        self.get_logger().info(
            f'SearchAndDetect mission started '
            f'(wall_standoff: {self.wall_standoff}m, '
            f'wall_spacing: {self.wall_spacing}m, '
            f'sweep_spacing: {self.sweep_spacing}m)'
        )

    def on_marker_confirmed(self, distance):
        if self.state == State.FINISHED:
            return

        self.marker_found = True
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None

        self.state = State.FINISHED
        result = String()
        result.data = 'marker_found'
        self.result_pub.publish(result)
        self.get_logger().info(f'Marker found at {distance:.2f}m. Mission complete.')

    def map_callback(self, msg):
        if self.state != State.WAITING_FOR_MAP:
            return

        self.grid = msg
        self.get_logger().info('Map received. Generating wall-following waypoints.')

        self.waypoints = generate_wall_following_waypoints(
            msg,
            standoff=self.wall_standoff,
            spacing=self.wall_spacing,
            min_clearance=self.min_clearance,
        )

        self.get_logger().info(f'Generated {len(self.waypoints)} wall-following waypoints.')

        if not self.waypoints:
            self.get_logger().warn('No wall waypoints. Skipping to lawnmower.')
            self.start_lawnmower(covered_mask=None)
            return

        self.state = State.WALL_FOLLOWING
        self.current_waypoint_idx = 0
        self.publish_waypoint_markers()
        self.navigate_to_waypoint()

    def start_lawnmower(self, covered_mask):
        self.get_logger().info('Starting lawnmower phase.')

        self.waypoints = generate_lawnmower_waypoints(
            self.grid,
            sweep_spacing=self.sweep_spacing,
            min_clearance=self.min_clearance,
            covered_mask=covered_mask,
        )

        self.get_logger().info(f'Generated {len(self.waypoints)} lawnmower waypoints.')

        if not self.waypoints:
            self.get_logger().warn('No lawnmower waypoints. Search complete.')
            self.finish('search_complete')
            return

        self.state = State.LAWNMOWER
        self.current_waypoint_idx = 0
        self.publish_waypoint_markers()
        self.navigate_to_waypoint()

    def publish_waypoint_markers(self):
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        msg.markers.append(delete_marker)

        is_wall = self.state == State.WALL_FOLLOWING
        color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.8) if is_wall else ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8)

        for i, (wx, wy, yaw) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position = Point(x=wx, y=wy, z=0.1)
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale = Vector3(x=0.3, y=0.08, z=0.08)
            m.color = color
            msg.markers.append(m)

        self.marker_pub.publish(msg)
        label = 'wall-following' if is_wall else 'lawnmower'
        self.get_logger().info(f'Published {len(self.waypoints)} {label} markers to /search/waypoints')

    def update_current_marker(self):
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if self.current_waypoint_idx > 0:
            prev = self.current_waypoint_idx - 1
            wx, wy, yaw = self.waypoints[prev]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'waypoints'
            m.id = prev
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position = Point(x=wx, y=wy, z=0.1)
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale = Vector3(x=0.3, y=0.08, z=0.08)
            m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)
            msg.markers.append(m)

        if self.current_waypoint_idx < len(self.waypoints):
            wx, wy, yaw = self.waypoints[self.current_waypoint_idx]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'waypoints'
            m.id = self.current_waypoint_idx
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position = Point(x=wx, y=wy, z=0.1)
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale = Vector3(x=0.5, y=0.12, z=0.12)
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)
            msg.markers.append(m)

        self.marker_pub.publish(msg)

    def navigate_to_waypoint(self):
        if self.marker_found or self.state == State.FINISHED:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            if self.state == State.WALL_FOLLOWING:
                self.get_logger().info('Wall-following complete. Computing coverage.')
                data, res, ox, oy = occupancy_to_numpy(self.grid)
                covered = mark_covered(
                    self.waypoints, self.sensor_range,
                    data.shape, res, ox, oy,
                )
                self.start_lawnmower(covered_mask=covered)
            else:
                self.get_logger().warn('Lawnmower complete. Marker not found.')
                self.finish('search_complete')
            return

        self.update_current_marker()
        wx, wy, yaw = self.waypoints[self.current_waypoint_idx]
        phase = 'Wall' if self.state == State.WALL_FOLLOWING else 'Lawn'
        self.get_logger().info(
            f'[{phase}] Waypoint {self.current_waypoint_idx + 1}/'
            f'{len(self.waypoints)}: ({wx:.1f}, {wy:.1f}, yaw={yaw:.2f})'
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint rejected, skipping.')
            self.current_waypoint_idx += 1
            self.navigate_to_waypoint()
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        self.current_goal_handle = None
        if self.marker_found or self.state == State.FINISHED:
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == State.WALL_FOLLOWING and self.pause_duration > 0:
                prev_state = self.state
                self.state = State.PAUSING
                self.pause_timer = self.create_timer(
                    self.pause_duration, lambda: self.on_pause_done(prev_state)
                )
            else:
                self.current_waypoint_idx += 1
                self.navigate_to_waypoint()
        else:
            self.get_logger().warn(f'Navigation failed (status {status}), skipping.')
            self.current_waypoint_idx += 1
            self.navigate_to_waypoint()

    def on_pause_done(self, prev_state):
        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None

        if self.marker_found or self.state == State.FINISHED:
            return

        self.state = prev_state
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
        self.get_logger().info(f'Search finished: {result_str}')


def main(args=None):
    rclpy.init(args=args)
    node = SearchAndDetect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
