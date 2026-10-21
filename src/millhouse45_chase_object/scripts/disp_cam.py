#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 3 - disp_cam (LAPTOP ONLY - debugging)
"""

# Unlike lab 2, the robot already does the detection and draws on the frame.
# This just shows whatever detect_object publishes.

import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

DEBUG_IMG_TOPIC = '/obj_finder/compressed'

# Define QOS profile for the debug imagae topic. Just a standard profile
CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def main():
    rclpy.init()
    node = rclpy.create_node('disp_cam')
    bridge = CvBridge()

    def image_callback(img_msg):
        cv2.imshow("obj_finder", bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8'))
        cv2.waitKey(1)

    node.create_subscription(CompressedImage, DEBUG_IMG_TOPIC, image_callback, CUSTOM_QOS_PROFILE)

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
