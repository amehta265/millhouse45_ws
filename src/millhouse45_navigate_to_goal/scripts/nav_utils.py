"""
Brian Huntley
Ankit Mehta
Lab 4 - nav_utils
"""

# Shared pieces used by more than one Lab 4 node.

import math
import rclpy
from geometry_msgs.msg import Vector3

# Look-ahead distance l (m)
LOOKAHEAD_L = 0.10

# One rotation matrix R(angle) = [[cos, -sin], [sin, cos]]
def rotate(x, y, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return c * x - s * y, s * x + c * y

# Robot (local) frame -> global frame:  R(theta) v.
def local_to_global(x, y, theta):
    return rotate(x, y, theta)

# Global frame -> robot (local) frame:
def global_to_local(x, y, theta):
    return rotate(x, y, -theta)

def to_vector3(x, y):
    return Vector3(x=float(x), y=float(y), z=0.0)

def seconds_now(node):
    return node.get_clock().now().nanoseconds * 1e-9

def run_node(node_class):
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and hasattr(node, 'stop_robot'):
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
