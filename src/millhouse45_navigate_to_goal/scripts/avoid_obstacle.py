#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - avoid_obstacle
"""

# behavior: "which direction gets me away from the obstacle?"

# Lecture equations:
#   error to the obstacle:   e = x_O - x where x is the look ahead point
#   avoid obstacle vector:   u_AO = -K(||e||) e     (go to goal with the direction flipped)
#   K = (1 / ||e||) * (c / (||e||^2 + eps))

# Stability note: flipping the sign gives e_dot = +K e, so on its own
# this behavior pushes the robot off to infinity. That is fine because:
#   1. we only use it near an obstacle

#   /obstacle (Obstacle) + /global_pose (RobotPose)  -->  avoid_obstacle  -->  /u_ao (Vector3, global frame)

import math
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from millhouse45_navigate_to_goal.msg import Obstacle, RobotPose
from nav_utils import local_to_global, to_vector3, run_node


def ao_vector(obs_x_local, obs_y_local, theta):
    C = 0.015    # ||u_AO|| = 0.15 m/s at 0.30 m from the obstacle, ~0.21 m/s at 0.25 m
    EPS = 0.01   # caps ||u_AO|| at C / EPS when the obstacle is right on top of us

    # /obstacle is in the robot frame, everything else is global
    ex, ey = local_to_global(obs_x_local, obs_y_local, theta)
    e_norm = math.hypot(ex, ey)
    if e_norm < 1e-6:
        return 0.0, 0.0
    K = (1.0 / e_norm) * (C / (e_norm ** 2 + EPS))
    return -K * ex, -K * ey


class AvoidObstacle(Node):
    def __init__(self):
        super().__init__('avoid_obstacle')
        self.theta = None   # latest robot heading in the global frame

        self.u_publisher = self.create_publisher(Vector3, '/u_ao', 5)
        self.create_subscription(RobotPose, '/global_pose', self.pose_callback, 5)
        self.create_subscription(Obstacle, '/obstacle', self.obstacle_callback, 5)

    def pose_callback(self, pose):
        self.theta = pose.theta

    def obstacle_callback(self, obs):
        # Publish on every lidar update. No obstacle (or no pose yet) -> zero vector.
        u_ao = (0.0, 0.0)
        if obs.found and self.theta is not None:
            u_ao = ao_vector(obs.x, obs.y, self.theta)
        self.u_publisher.publish(to_vector3(*u_ao))


if __name__ == '__main__':
    run_node(AvoidObstacle)
