#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 5 - waypoint_navigator
"""

# Sends the lab waypoints to Nav2 one after the other (map frame), waiting for each one to finish.
#
# Click mode (no --wp): click the waypoints in RViz with "Publish Point", in order, then press Enter.
#   ros2 run turtlebot3_recovery_lab waypoint_navigator.py
#
# Typed mode:
#   ros2 run turtlebot3_recovery_lab waypoint_navigator.py --wp 1.2,0.4 --wp 2.0,1.1 --wp 0.3,1.5

import argparse
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clicked = []

    def on_click(self, msg):
        self.clicked.append((msg.point.x, msg.point.y))
        self.get_logger().info(f'Waypoint {len(self.clicked)}: ({msg.point.x:.2f}, {msg.point.y:.2f})')

    def collect_clicks(self):
        """Record RViz "Publish Point" clicks until Enter is pressed."""
        sub = self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        enter = threading.Event()
        threading.Thread(target=lambda: (input(), enter.set()), daemon=True).start()
        self.get_logger().info('Click the waypoints in RViz with "Publish Point", in order. Press Enter to start.')
        while rclpy.ok() and not enter.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return self.clicked

    def go_to(self, x, y):
        """Send one goal and wait until Nav2 finishes it. True if the robot got there."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wp', action='append', default=[], metavar='X,Y',
                        help='waypoint in the map frame, repeat once per waypoint, in order. '
                             'Leave out to click the waypoints in RViz instead')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    waypoints = [tuple(float(v) for v in wp.split(',')) for wp in args.wp]

    rclpy.init()
    node = WaypointNavigator()
    node.get_logger().info('Waiting for Nav2 (navigate_to_pose) ...')
    node.client.wait_for_server()

    try:
        if not waypoints:
            waypoints = node.collect_clicks()
        if not waypoints:
            node.get_logger().warn('No waypoints, nothing to do.')
        for i, (x, y) in enumerate(waypoints, start=1):
            node.get_logger().info(f'Waypoint {i}: going to ({x:.2f}, {y:.2f})')
            if node.go_to(x, y):
                node.get_logger().info(f'Waypoint {i}: reached')
            else:
                node.get_logger().warn(f'Waypoint {i}: failed, moving on')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
