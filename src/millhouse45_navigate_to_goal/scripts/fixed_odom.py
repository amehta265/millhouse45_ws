#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - fixed_odom
"""

# Turns raw /odom into the GLOBAL pose
#
# We use the Rotation_Script.py logic ( func update_Odometry) in this node. We then publish the result,
# so every other node gets the same corrected pose.

import math
from rclpy.node import Node
from nav_msgs.msg import Odometry

from millhouse45_navigate_to_goal.msg import RobotPose
from nav_utils import global_to_local, run_node


def z_from_odom_quaternion(q):
    # Orientation in /odom is a quaternion (from lab material). We only care about rotation about z axis
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FixedOdom(Node):
    def __init__(self):
        super().__init__('fixed_odom')

        self.Init = True
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_angle = 0.0

        self.pose_publisher = self.create_publisher(RobotPose, '/global_pose', 5)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 5)

    def odom_callback(self, odom):
        position = odom.pose.pose.position
        orientation = z_from_odom_quaternion(odom.pose.pose.orientation)

        if self.Init:
            # We want to remember the first reading. Everything after this is measured relative to it.
            self.Init = False
            self.init_angle = orientation
            self.init_x, self.init_y = global_to_local(position.x, position.y, self.init_angle)
            self.get_logger().info(
                f'Saved start pose from /odom: x={position.x:.3f} y={position.y:.3f} '
                f'z={math.degrees(orientation):.1f} deg. This is now (0, 0, 0).')

        # Rotate /odom into the frame the robot was facing at the start (R(-init_angle), same matrix as
        # rotation_script), then subtract the start. Now the start of the run is (0, 0) facing +x.
        rx, ry = global_to_local(position.x, position.y, self.init_angle)

        # Heading relative to the start, wrapped to [-pi, pi] with the atan2(sin, cos) trick from lecture
        theta = orientation - self.init_angle

        pose = RobotPose()
        pose.x = float(rx - self.init_x)
        pose.y = float(ry - self.init_y)
        pose.theta = float(math.atan2(math.sin(theta), math.cos(theta)))
        self.pose_publisher.publish(pose)

        self.get_logger().info(
            f'global pose x={pose.x:+.3f} y={pose.y:+.3f} theta={math.degrees(pose.theta):+.1f} deg',
            throttle_duration_sec=1.0)


if __name__ == '__main__':
    run_node(FixedOdom)
