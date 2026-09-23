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

        # If the ball is within DEADBAND_PX of the center it counts as being in front, so don't move
        if msg.found:
            offset = msg.obj_center_x - msg.image_center_x   # positive = ball is right of center
            if abs(offset) > 30:
                # So looks like +angular.z = counter-clockwise (left), -angular.z = clockwise (right)
                # Ball on the right would mean that the offset > 0 so negative z so it shall turn right. Same thing the other way round
                # Also want the robot to slow down as it think it is approaching the ball so it dont overshoot
                ang = -0.003 * offset
                twist.angular.z = max(-0.5, min(0.5, ang))

        cmd_vel_publisher.publish(twist)

    node.create_subscription(ObjInfo, OBJ_INFO_TOPIC, obj_callback, CUSTOM_QOS_PROFILE)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
