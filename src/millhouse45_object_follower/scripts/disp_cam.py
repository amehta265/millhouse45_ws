#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
"""
import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from detector import detect_object

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def disp_cam():
    rclpy.init()
    node = rclpy.create_node('disp_cam')
    bridge = CvBridge()

    # All the below logic is pulled from lab 1 and added here
    def image_callback(image_message):
        # Camera is open at this point
        frame = bridge.compressed_imgmsg_to_cv2(image_message, 'bgr8')
        coords, radius, mask = detect_object(frame)

        overlay = frame.copy()
        if coords is not None:
            # Draw bounding box
            cx, cy = int(round(coords[0])), int(round(coords[1]))
            color = (0, 255, 0)
            cv2.circle(overlay, (cx, cy), int(round(radius)), color, 2)
            cv2.circle(overlay, (cx, cy), 4, color, -1)

            label = (f"cx: {cx}, cy: {cy}")
        else:
            label = "no ball"

        cv2.putText(overlay, f"{label}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("frame", overlay)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

    node.create_subscription(CompressedImage, '/image_raw/compressed', image_callback, CUSTOM_QOS_PROFILE)

    try:
        rclpy.spin(node)  # keep the node alive so image_callback runs
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    disp_cam()
