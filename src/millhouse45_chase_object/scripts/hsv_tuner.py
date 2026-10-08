#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 3 - hsv_tuner (LAPTOP ONLY - for tuning the colour thresholds, NOT part of the demo)
"""

# Shows the robot camera with sliders for the HSV thresholds, using the exact same detector code the robot runs.
# Move the sliders until ONLY the ball is white in the mask (and legs / clothes / chairs are black),
# then press 'p' to print the values and paste them into the HSV dict at the top of detector.py.
#
# Run on the laptop while camera_robot.launch.py is running on the robot:
#   ros2 run millhouse45_chase_object hsv_tuner.py
# Keys: p = print current values, q = quit

import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import detector

WINDOW = 'hsv_tuner'

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def main():
    rclpy.init()
    node = rclpy.create_node('hsv_tuner')
    bridge = CvBridge()

    # One slider per threshold, starting from whatever is in detector.py right now
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    for key, value in detector.HSV.items():
        max_value = 179 if key.startswith('h') else 255
        cv2.createTrackbar(key, WINDOW, value, max_value, lambda _: None)

    def read_sliders():
        return {key: cv2.getTrackbarPos(key, WINDOW) for key in detector.HSV}

    def print_values(hsv):
        print('\nPaste this over the HSV dict in detector.py:')
        print('HSV = {')
        print(f"    'h1_lo': {hsv['h1_lo']},   'h1_hi': {hsv['h1_hi']},")
        print(f"    'h2_lo': {hsv['h2_lo']}, 'h2_hi': {hsv['h2_hi']},")
        print(f"    's_lo': {hsv['s_lo']},   's_hi': {hsv['s_hi']},")
        print(f"    'v_lo': {hsv['v_lo']},   'v_hi': {hsv['v_hi']},")
        print('}', flush=True)

    def image_callback(img_msg):
        frame = bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
        hsv = read_sliders()
        masked_img, candidates = detector.find_candidates(frame, hsv)
        ball = detector.pick_ball(candidates)
        label = 'ball' if ball is not None else 'no ball'
        view = detector.draw_debug(frame, masked_img, candidates, ball, label)
        # Robot images are small (320x240) - blow them up so they're easier to see
        view = cv2.resize(view, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(WINDOW, view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'):
            print_values(hsv)
        elif key == ord('q'):
            print_values(hsv)
            raise KeyboardInterrupt

    node.create_subscription(CompressedImage, '/image_raw/compressed', image_callback, CUSTOM_QOS_PROFILE)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
