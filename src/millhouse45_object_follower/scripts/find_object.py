#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
"""
#!/usr/bin/env python3
# Run on the robot - image in, object location out. Also publishes the processed image debug
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from .detector import detect_object
from millhouse45_object_follower.msg import ObjInfo

OBJ_INFO_TOPIC = '/obj/info'

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def main():
    rclpy.init()
    node = rclpy.create_node('find_object')
    bridge = CvBridge()
    obj_info_publisher = node.create_publisher(ObjInfo, OBJ_INFO_TOPIC, 5)

    def image_callback(img_msg):
        # Basically converting sensor_msgs/compressed image to an openCV version the image in bgr8 like in lab 1
        frame = bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')

        # Trying to re-use lab 1 detector. Don't need the rotate and mask anymore cause that was being used for debugging
        # disp_cam should have its own debugging stuff now 
        coords, _, _ = detect_object(frame)

        msg = ObjInfo()
        msg.found = coords is not None
        # If you find the obj
        if msg.found:
            msg.obj_center_x = float(coords[0])
        msg.image_center_x = frame.shape[1] / 2.0   # Kinda assuming that the images center is the width img / 2
        obj_info_publisher.publish(msg)

    # In order to get image data
    node.create_subscription(CompressedImage, '/image_raw/compressed', image_callback, CUSTOM_QOS_PROFILE)
    rclpy.spin(node)
           
if __name__ == '__main__':
    main()









