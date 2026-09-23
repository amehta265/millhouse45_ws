#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
"""

# Run on the robot - receives the object location and outputs a TWIST command

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist

from millhouse45_object_follower.msg import ObjInfo

OBJ_INFO_TOPIC = '/obj/info'
CMD_VEL_TOPIC = '/cmd_vel'

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

def main():
    rclpy.init()
    node = rclpy.create_node('rotate_robot')
    cmd_vel_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 5)

    def obj_callback(msg):
        twist = Twist()   # all zeros = stay in place

        # I'm making the assumption that if the ball is within 20px of the center I'm counting that as being in the middle so dont move
        if msg.found:
            offset = msg.obj_center_x - msg.image_center_x   # positive = ball is right of center
            if offset > 20:
                twist.angular.z = -0.3     # turn right
            elif offset < -20:
                twist.angular.z = -0.3      # turn left

        cmd_vel_publisher.publish(twist)

    node.create_subscription(ObjInfo, OBJ_INFO_TOPIC, obj_callback, CUSTOM_QOS_PROFILE)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
