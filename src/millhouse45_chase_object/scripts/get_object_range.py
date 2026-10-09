#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 3 - get_object_range
"""

# Run on the robot - camera tells us WHICH direction the ball is in, lidar tells us HOW FAR.
# We just keep the newest scan around and every time a bearing comes in we look up the lidar beams pointing that way.

import math
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

from millhouse45_chase_object.msg import ObjLocation

BEARING_TOPIC = '/obj/bearing'
RANGE_TOPIC = '/obj/range'

# How many degrees either side of the camera bearing we check on the lidar.
# The ball is small so a few degrees is enough, and it covers the small offset that may exist between camera and lidar.
WINDOW_DEG = 5.0

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def range_at_bearing(scan, bearing, window_rad):
    # The lidar gives us one long list of distances (scan.ranges), one per beam, sweeping counter-clockwise.
    # Beam 0 points straight ahead - 0 rad at scan.angle_min, and each next beam comes along at
    # scan.angle_increment radians. So: angle of beam i = angle_min + (i * angle_increment)
    # So the goal is to convert each bearing reading to an index so we can get the appropriate section.
    num_beams = len(scan.ranges)

    # Step 1: which beam points at the ball?
    # i = (bearing - angle_min) / angle_increment
    angle_from_first_beam = bearing - scan.angle_min
    lidar_index_of_object = int(round(angle_from_first_beam / scan.angle_increment))

    # Step 2: need to calculate the number of beams to check on either side just to be careful we have covered all of the camera's bases.
    beams_each_side = max(1, int(window_rad / scan.angle_increment))

    first_index = lidar_index_of_object - beams_each_side
    last_index = lidar_index_of_object + beams_each_side

    # Step 3: collect the readings in that window, skipping bad ones
    valid_ranges = []
    for i in range(first_index, last_index + 1):
        # Ball on the right = negative bearing = negative index. Python's % wraps it to the end of the list,
        # e.g. index -3 -> num_beams - 3, which is the beam just to the right of straight ahead
        wrapped_index = i % num_beams
        distance = scan.ranges[wrapped_index]

        # The lidar reports 0 or inf when a beam doesn't hit anything (and NaN fails this check too)
        if scan.range_min < distance < scan.range_max:
            valid_ranges.append(distance)

    if len(valid_ranges) == 0:
        return None

    # min reading corresponds to the object under consideration as other readings may be the background / wall that are further away
    return min(valid_ranges)

def main():
    rclpy.init()
    node = rclpy.create_node('get_object_range')
    range_publisher = node.create_publisher(ObjLocation, RANGE_TOPIC, 5)

    latest = {'scan': None}

    def scan_callback(scan):
        latest['scan'] = scan

    def bearing_callback(b):
        msg = ObjLocation()
        msg.found = False
        msg.distance = float('nan')

        if b.found and latest['scan'] is not None:
            dist = range_at_bearing(latest['scan'], b.bearing, math.radians(WINDOW_DEG))
            if dist is not None:
                msg.found = True
                msg.bearing = b.bearing
                msg.distance = float(dist)
                node.get_logger().info(f"ball at {math.degrees(b.bearing):+.1f} deg, {dist:.2f} m",
                                       throttle_duration_sec=0.5)

        range_publisher.publish(msg)

    node.create_subscription(LaserScan, '/scan', scan_callback, CUSTOM_QOS_PROFILE)
    node.create_subscription(ObjLocation, BEARING_TOPIC, bearing_callback, CUSTOM_QOS_PROFILE)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
