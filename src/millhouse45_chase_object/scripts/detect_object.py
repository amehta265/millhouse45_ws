#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 3 - detect_object
"""

# Run on the robot - basically lab 2's find_object, but instead of sending pixels we send a bearing in radians
# so the lidar node can use it i.e. we look up the distance for the provided bearing index in the lidar array.
# It also publishes the processed frame so disp_cam on the laptop can show it for debugging

import math
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from detector import find_candidates, pick_ball, draw_debug
from millhouse45_chase_object.msg import ObjLocation

BEARING_TOPIC = '/obj/bearing'
DEBUG_IMG_TOPIC = '/obj_finder/compressed'

# Horizontal field of view of the camera = how wide an angle the image covers, edge to edge.
# 62.2 deg is the Pi Camera v2 spec.
HFOV_DEG = 62.2

# If the ball disappears for just a frame or two (motion blur while turning, a hand passing in front),
# keep reporting where it last was instead of telling chase_object to stop. ~30 fps -> 3 frames is ~0.1 s.
# Longer than that and we say "not found" for real
HOLD_FRAMES = 3

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def main():
    rclpy.init()
    node = rclpy.create_node('detect_object')
    bridge = CvBridge()
    bearing_publisher = node.create_publisher(ObjLocation, BEARING_TOPIC, 5)
    debug_publisher = node.create_publisher(CompressedImage, DEBUG_IMG_TOPIC, 1)

    # Where the ball was last time (x, y, r) and how many frames in a row we've missed it
    track = {'last': None, 'missed': 0}

    def image_callback(img_msg):
        frame = bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
        width = frame.shape[1]

        # Find every red blob, then pick the round one - preferring the one near where the ball just was
        masked_img, candidates = find_candidates(frame)
        ball = pick_ball(candidates, track['last'])

        if ball is not None:
            track['last'] = (ball['x'], ball['y'], ball['r'])
            track['missed'] = 0
            coords = (ball['x'], ball['y'])
        else:
            track['missed'] += 1
            if track['last'] is not None and track['missed'] <= HOLD_FRAMES:
                # Short dropout - reuse the last position
                coords = (track['last'][0], track['last'][1])
            else:
                # Really lost it - forget the old position so we don't lock onto something near it later
                track['last'] = None
                coords = None

        # Same assumption as lab 2 - the image center is half the width
        image_center_x = width / 2.0

        msg = ObjLocation()
        # Camera alone can't tell how far the ball is. ROS messages have no "null", and leaving it at the
        # default 0.0 would look like the ball is touching the robot, so NaN = "not measured"
        msg.distance = float('nan')
        msg.found = coords is not None
        if msg.found:
            # Degrees per pixel: the image is HFOV_DEG wide, so each pixel off-center is worth the same slice of angle.
            # offset_fraction: -1 at the right edge, 0 at the center, +1 at the left edge
            # (center - x, not x - center, because pixels grow to the right but in ROS + angle is to the LEFT)
            offset_fraction = (image_center_x - coords[0]) / image_center_x
            half_fov_rad = math.radians(HFOV_DEG) / 2.0
            msg.bearing = offset_fraction * half_fov_rad   # left edge -> +31.1 deg, center -> 0, right edge -> -31.1 deg
        bearing_publisher.publish(msg)

        # Only bother drawing + compressing the debug image if disp_cam is actually listening (saves the Pi some work)
        # Shows camera + mask side by side: green circle = ball we picked, red outlines = red blobs we rejected
        # (numbers on them are circularity/fill - handy for tuning MIN_CIRCULARITY / MIN_FILL in detector.py)
        if debug_publisher.get_subscription_count() > 0:
            if ball is not None:
                label = f"bearing: {math.degrees(msg.bearing):+.1f} deg"
            elif msg.found:
                label = f"holding last ({track['missed']})"
            else:
                label = "no ball"
            debug_img = draw_debug(frame, masked_img, candidates, ball, label)
            debug_publisher.publish(bridge.cv2_to_compressed_imgmsg(debug_img))

    node.create_subscription(CompressedImage, '/image_raw/compressed', image_callback, CUSTOM_QOS_PROFILE)

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
