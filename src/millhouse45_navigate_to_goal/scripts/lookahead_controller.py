#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - lookahead_controller
"""

# turns the chosen velocity vector u = [x_dot, y_dot] into a Twist (v, w).
#
# We are replacing the two PID loops from Lab 3 with look ahead dynamics i.e. near identity diffeomorphism

#   Pick a point a distance l in front of the robot, on the line through the middle of the axle:
#     x~ = x + l cos(theta),   y~ = y + l sin(theta)
#   Take the time derivative and plug in the unicycle model:
#     x~_dot = v cos(theta) - l w sin(theta)
#     y~_dot = v sin(theta) + l w cos(theta)
#   In matrix form:  [x~_dot, y~_dot]^T = R(theta) diag(1, l) [v, w]^T
#   That matrix is invertible whenever l != 0, so for ANY u we want the look-ahead point to move with:
#     [v, w]^T = diag(1, 1/l) R(-theta) u

#   /u_cmd (Vector3, global frame) + /global_pose (RobotPose)  -->  lookahead_controller  -->  /cmd_vel (Twist)

from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from millhouse45_navigate_to_goal.msg import RobotPose
from nav_utils import LOOKAHEAD_L, global_to_local, seconds_now, run_node


def u_to_cmd_vel(ux, uy, theta):
    """[v, w] = diag(1, 1/l) R(-theta) u, then scaled down so no wheel goes over its limit."""
    MAX_LIN_VEL = 0.20     # m/s. Burger max is 0.22.
    MAX_ANG_VEL = 1.5      # rad/s.
    WHEEL_RADIUS = 0.033   # R (m)
    AXLE_LENGTH = 0.160    # b (m)

    # Look-ahead mapping. R(-theta) u = u in the robot frame = (part along the heading, part to the left)
    u_forward, u_left = global_to_local(ux, uy, theta)
    v = u_forward
    w = u_left / LOOKAHEAD_L

    # Wheel speeds from lecture: phi_R = (2v + wb) / 2R, phi_L = (2v - wb) / 2R.
    # If either wheel (or w) is over its limit, shrink v and w by the SAME factor -> same direction, just slower.
    max_wheel = MAX_LIN_VEL / WHEEL_RADIUS                      # e.g. 0.20 / 0.033 = 6.06 rad/s
    phi_r = (2.0 * v + w * AXLE_LENGTH) / (2.0 * WHEEL_RADIUS)
    phi_l = (2.0 * v - w * AXLE_LENGTH) / (2.0 * WHEEL_RADIUS)

    scale = 1.0
    fastest_wheel = max(abs(phi_r), abs(phi_l))
    if fastest_wheel > max_wheel:
        scale = min(scale, max_wheel / fastest_wheel)
    if abs(w) > MAX_ANG_VEL:
        scale = min(scale, MAX_ANG_VEL / abs(w))
    return v * scale, w * scale


class LookaheadController(Node):
    def __init__(self):
        super().__init__('lookahead_controller')
        self.theta = None
        self.last_cmd_time = None   # None = we are stopped

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.create_subscription(RobotPose, '/global_pose', self.pose_callback, 5)
        self.create_subscription(Vector3, '/u_cmd', self.u_cmd_callback, 5)
        self.create_timer(0.1, self.watchdog)

    def pose_callback(self, pose):
        self.theta = pose.theta

    def u_cmd_callback(self, u):
        if self.theta is None:
            return   # no heading yet, cannot rotate u into the robot frame

        v, w = u_to_cmd_vel(u.x, u.y, self.theta)
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_publisher.publish(twist)
        self.last_cmd_time = seconds_now(self)

    def watchdog(self):
        # Same as Lab 3: if the behavior manager goes quiet for 0.5 s (node died, etc.), stop the robot.
        if self.last_cmd_time is not None and seconds_now(self) - self.last_cmd_time > 0.5:
            self.stop_robot()
            self.last_cmd_time = None   # only fire once
            self.get_logger().warn('No /u_cmd for a while, stopping the robot.')

    def stop_robot(self):
        # Also called by run_node on Ctrl-C
        self.cmd_vel_publisher.publish(Twist())


if __name__ == '__main__':
    run_node(LookaheadController)
