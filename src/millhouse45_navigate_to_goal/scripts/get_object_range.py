#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - get_object_range
"""

# Finds the closest obstacle point in the lidar scan and publishes the vector to it.

# Steps for every scan:
#   1. Filter: throw away NaN, inf, 0 + readings outside the
#      sensor's valid range. Also throw away "lonely" readings that have no neighbour beam at a similar distance. Cause
#      this is more likely noise than a real box.
#   2. Only look closer than MAX_DETECT_RANGE. We dont care about things further than that.
#      We check full 360 degrees.
#   3. Turn each (distance, angle) into a Euclidean vector.
#   4. Keep the shortest vector.
#
# All the calculations are done from the look ahead point determined from the look ahead dynamics
# The result is in the robot's LOCAL frame. avoid_obstacle rotates it into the global frame.

#   /scan (sensor_msgs/LaserScan)  -->  get_object_range  -->  /obstacle (Obstacle: found, x, y, distance, angle)

import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

from millhouse45_navigate_to_goal.msg import Obstacle
from nav_utils import LOOKAHEAD_L, run_node

SCAN_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)


def nearest_obstacle(scan):
    """Return (found, x, y, distance, angle) of the closest believable lidar point.
    Robot frame, measured from the look-ahead point."""
    MAX_DETECT_RANGE = 0.8       # m. Has to be bigger than the manager's SAFE_DISTANCE + EPSILON
    NEIGHBOUR_TOLERANCE = 0.05   # m. neighboring beams must be at least this close together = obstacle
    LIDAR_X_OFFSET = -0.032      # m. lidar sits 3.2 cm behind the point /odom tracks (Burger URDF)

    ranges = scan.ranges
    n = len(ranges)
    best = None   # (distance, x, y)

    def is_valid(r):
        # NaN, inf and 0 all fail this
        return scan.range_min < r < scan.range_max

    for i in range(n):
        r = ranges[i]

        # remove bad readings and points too far away to care about
        if not is_valid(r) or r > MAX_DETECT_RANGE:
            continue

        # At least one neighbouring beam has to see something at roughly the same distance.
        left = ranges[(i + 1) % n] # wrap around
        right = ranges[(i - 1) % n]
        has_neighbour = ((is_valid(left) and abs(left - r) < NEIGHBOUR_TOLERANCE) or
                         (is_valid(right) and abs(right - r) < NEIGHBOUR_TOLERANCE))
        if not has_neighbour:
            continue

        # polar -> Euclidean (this vector starts at the lidar)
        beam_angle = scan.angle_min + i * scan.angle_increment
        x = r * math.cos(beam_angle) + LIDAR_X_OFFSET - LOOKAHEAD_L # Need to account for where lidar is and the fact
        # that calculations are based off of look ahead dynamics
        y = r * math.sin(beam_angle)
        d = math.hypot(x, y)

        # keep the closest
        if best is None or d < best[0]:
            best = (d, x, y)

    if best is None:
        return False, 0.0, 0.0, math.inf, 0.0

    d, x, y = best
    return True, x, y, d, math.atan2(y, x)   # atan2 keeps the angle in [-pi, pi]


class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')
        self.obstacle_publisher = self.create_publisher(Obstacle, '/obstacle', 5)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, SCAN_QOS)

    def scan_callback(self, scan):
        found, x, y, d, angle = nearest_obstacle(scan)

        msg = Obstacle()
        msg.found = found
        msg.x = float(x)
        msg.y = float(y)
        msg.distance = float(d)
        msg.angle = float(angle)
        self.obstacle_publisher.publish(msg)

        if found:
            self.get_logger().info(f'nearest obstacle {d:.2f} m at {math.degrees(angle):+.0f} deg',
                                   throttle_duration_sec=0.5)


if __name__ == '__main__':
    run_node(GetObjectRange)
