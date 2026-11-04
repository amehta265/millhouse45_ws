#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - go_to_goal
"""

# The GO TO GOAL behavior: "which direction should I go, and how fast, to reach the goal?"

# Lecture equations:
#   error to the goal:        e = x_G - x
#   go to goal vector:        u_GTG = K(||e||) e
#   bounded gain (10_14):     K = v0 (1 - exp(-a ||e||^2)) / ||e||

#   /global_pose (RobotPose) + /current_goal (Point)  -->  go_to_goal  -->  /u_gtg (Vector3, global frame)

import math
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3

from millhouse45_navigate_to_goal.msg import RobotPose
from nav_utils import to_vector3, run_node


def gtg_vector(x, y, goal_x, goal_y):
    """u_GTG = K(||e||) e with K = v0 (1 - exp(-a ||e||^2)) / ||e||."""
    V0 = 0.18   # top speed of the GTG vector (m/s). Burger max is 0.22, keep some margin.
    A = 40.0    # how quickly we reach v0 as we get further from the goal. Further away = more acc - vice versa

    ex = goal_x - x
    ey = goal_y - y
    e_norm = math.hypot(ex, ey)
    if e_norm < 1e-6:
        return 0.0, 0.0   # sitting on the goal: nothing to do (and avoid dividing by zero)
    K = V0 * (1.0 - math.exp(-A * e_norm ** 2)) / e_norm
    return K * ex, K * ey


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.goal = None   # (x, y). None until the behavior manager tells us the first goal.

        self.u_publisher = self.create_publisher(Vector3, '/u_gtg', 5)
        self.create_subscription(Point, '/current_goal', self.goal_callback, 5)
        self.create_subscription(RobotPose, '/global_pose', self.pose_callback, 5)

    def goal_callback(self, goal):
        self.goal = (goal.x, goal.y)

    def pose_callback(self, pose):
        # Recompute u every time the pose updates (odometry is our fastest sensor).
        if self.goal is None:
            return
        self.u_publisher.publish(to_vector3(*gtg_vector(pose.x, pose.y, self.goal[0], self.goal[1])))


if __name__ == '__main__':
    run_node(GoToGoal)
