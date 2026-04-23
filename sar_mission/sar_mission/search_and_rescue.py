#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from sar_mission.explore_and_detect import ExploreAndDetect
from sar_mission.search_and_detect import SearchAndDetect


class Phase(Enum):
    EXPLORE = auto()
    SEARCH = auto()
    DONE = auto()


class SearchAndRescue(Node):
    def __init__(self):
        super().__init__('search_and_rescue')

        self.phase = Phase.EXPLORE
        self.search_node = None

        self.result_sub = self.create_subscription(
            String, '/mission/result', self.result_callback, 10
        )

        self.get_logger().info('SearchAndRescue orchestrator started.')

    def result_callback(self, msg):
        if self.phase == Phase.DONE:
            return

        if self.phase == Phase.EXPLORE:
            if msg.data == 'marker_found':
                self.get_logger().info('Marker found during exploration. Mission complete!')
                self.phase = Phase.DONE
            elif msg.data == 'exploration_complete':
                self.get_logger().info(
                    'Exploration finished without finding marker. Starting search phase.'
                )
                self.phase = Phase.SEARCH
                self.search_node = SearchAndDetect()
                self.executor.add_node(self.search_node)

        elif self.phase == Phase.SEARCH:
            if msg.data == 'marker_found':
                self.get_logger().info('Marker found during search. Mission complete!')
                self.phase = Phase.DONE
            elif msg.data == 'search_complete':
                self.get_logger().warn('Search finished without finding marker. Mission failed.')
                self.phase = Phase.DONE

    def set_executor(self, executor):
        self.executor = executor


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    orchestrator = SearchAndRescue()
    orchestrator.set_executor(executor)
    explore_node = ExploreAndDetect()

    executor.add_node(orchestrator)
    executor.add_node(explore_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
