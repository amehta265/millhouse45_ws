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

        # If the ball is within deadband of the center it counts as being in front, so don't move. Chose 20 via trial and error
        if msg.found:
            # Ofsset is basically how many pixels away is the ball from the center of the img
            offset = msg.obj_center_x - msg.image_center_x   # positive = ball is right of center + vice versa
            if abs(offset) > 20:
                # So looks like +angular.z = counter-clockwise (left), -angular.z = clockwise (right). Incorrect signs did not work!
                # Ball on the right would mean that the offset > 0 so negative z so it shall turn right. Same thing the other way round

                # Also want the robot to slow down as it think it is approaching the ball so it dont overshoot
                # so turn speed is like proportional control. If obj is say 100px away from center then 100px * 0.03 = 0.3 rad/s
                # but if its a mere 40px off that would be 0.12rad/s i.e. slowing down!
                turn_speed = -0.003 * offset

                # Tryna cap the speed at 0.55
                twist.angular.z = max(-0.55, min(0.55, turn_speed))

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
