#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid

from sar_mission.marker_monitor import MarkerMonitor


class State(Enum):
    EXPLORING = auto()
    FINISHED = auto()


class ExploreAndDetect(Node):
    def __init__(self):
        super().__init__('explore_and_detect')

        self.declare_parameter('confirmation_duration', 1.0)
        self.declare_parameter('exploration_timeout', 30.0)

        self.confirmation_duration = self.get_parameter('confirmation_duration').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value

        self.state = State.EXPLORING
        self.last_map_change_time = None
        self.previous_map_data = None

        self.resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.result_pub = self.create_publisher(String, '/mission/result', 10)

        self.monitor = MarkerMonitor(
            self, self.confirmation_duration, self.on_marker_confirmed
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 1
        )

        self.exploration_check_timer = self.create_timer(5.0, self.check_exploration_done)

        self.get_logger().info(
            f'ExploreAndDetect mission started '
            f'(confirmation: {self.confirmation_duration}s, '
            f'exploration timeout: {self.exploration_timeout}s)'
        )

    def on_marker_confirmed(self, distance):
        if self.state == State.FINISHED:
            return

        self.stop_exploration()
        self.state = State.FINISHED

        result = String()
        result.data = 'marker_found'
        self.result_pub.publish(result)
        self.get_logger().info('Mission complete: marker found.')

    def map_callback(self, msg):
        if self.state != State.EXPLORING:
            return

        current_data = bytes(msg.data)
        now = self.get_clock().now()

        if self.previous_map_data is None or current_data != self.previous_map_data:
            self.last_map_change_time = now
            self.previous_map_data = current_data

    def check_exploration_done(self):
        if self.state != State.EXPLORING:
            return

        if self.last_map_change_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_map_change_time).nanoseconds / 1e9

        if elapsed >= self.exploration_timeout:
            self.get_logger().info(
                f'Map unchanged for {self.exploration_timeout}s. Exploration complete.'
            )
            self.exploration_check_timer.cancel()
            self.stop_exploration()
            self.state = State.FINISHED

            result = String()
            result.data = 'exploration_complete'
            self.result_pub.publish(result)
            self.get_logger().info('Mission complete: marker not found during exploration.')

    def stop_exploration(self):
        stop_msg = Bool()
        stop_msg.data = False
        self.resume_pub.publish(stop_msg)


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
