#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - follow_wall
"""

# "slide along the obstacle instead of running away".

# From lecture, we know that u_AO points straight away from the obstacle. We turn this
# 90 degrees so it points ALONG the obstacle's edge instead. Need to be careful which way we turn it
#  cause it has to be turned along the edge of the obstacle:
#   clockwise:          u_FW^c  = alpha R(-pi/2) u_AO
#   counter-clockwise:  u_FW^cc = alpha R(+pi/2) u_AO

# Example: obstacle straight ahead -> u_AO points backwards (-x).
#   R(-pi/2) turns it to +y (go left),  so the robot circles the obstacle clockwise.
#   R(+pi/2) turns it to -y (go right), so the robot circles the obstacle counter-clockwise.

# We publish BOTH. Choosing one is the behavior manager's job

#   /u_ao (Vector3)  -->  follow_wall  -->  /u_fw_c and /u_fw_cc (Vector3, global frame)

import math
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from nav_utils import rotate, to_vector3, run_node


def fw_vectors(u_ao_x, u_ao_y):
    V_FW = 0.12   # speed while sliding along the obstacle (m/s). Slower than GTG cause are close to something.

    norm = math.hypot(u_ao_x, u_ao_y)
    if norm < 1e-6:
        return (0.0, 0.0), (0.0, 0.0)   # no obstacle -> nothing to follow
    alpha = V_FW / norm   # alpha picked so the follow wall speed is always V_FW
    # Not a frame change, just turning the vector by -90 / +90 degrees, so plain rotate()
    cw = rotate(u_ao_x, u_ao_y, -math.pi / 2.0)
    ccw = rotate(u_ao_x, u_ao_y, +math.pi / 2.0)
    return (alpha * cw[0], alpha * cw[1]), (alpha * ccw[0], alpha * ccw[1])


class FollowWall(Node):
    def __init__(self):
        super().__init__('follow_wall')
        self.c_publisher = self.create_publisher(Vector3, '/u_fw_c', 5)
        self.cc_publisher = self.create_publisher(Vector3, '/u_fw_cc', 5)
        self.create_subscription(Vector3, '/u_ao', self.u_ao_callback, 5)

    def u_ao_callback(self, u_ao):
        cw, ccw = fw_vectors(u_ao.x, u_ao.y)
        self.c_publisher.publish(to_vector3(*cw))
        self.cc_publisher.publish(to_vector3(*ccw))


if __name__ == '__main__':
    run_node(FollowWall)
