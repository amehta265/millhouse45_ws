#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 3 - chase_object
"""

# Run on the robot - takes bearing + distance and outputs a TWIST with two controllers:
#   angular controller: drive the bearing to 0 (face the ball)          -> angular.z
#   linear controller:  drive (distance - desired distance) to 0         -> linear.x

# Building of the same idea as rotate_robot from lab 2 (that was just the P part of the angular controller in pixels).

import math
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist

from millhouse45_chase_object.msg import ObjLocation

RANGE_TOPIC = '/obj/range'
CMD_VEL_TOPIC = '/cmd_vel'

# Everything is a parameter so we can tune live without rebuilding. This helps with debugging e.g.
# ros2 param set /chase_object ang_kp 2.0
params = {
    'desired_distance': 0.5,     # m

    # Start with 0 on integral so essentially a PD controller
    'ang_kp': 1.5, 'ang_ki': 0.0, 'ang_kd': 0.1, #Defining tuning terms for angular controller
    'lin_kp': 1.0, 'lin_ki': 0.0, 'lin_kd': 0.05,# Defining tuning terms for linear controller
    'i_limit': 0.5, # Ceiling on integral term to prevent windup i.e. building up of past error

    'ang_deadband': 0.07,        #  close enough to facing the ball
    'lin_deadband': 0.03,        #  close enough to the desired distance
    'max_ang_vel': 1.0,          # (burger can do ~2.8 but that's scary)
    'max_lin_vel': 0.2,          # (burger max is 0.22)
    'timeout': 0.5,              # stop if we haven't heard about the ball in this long
}

CUSTOM_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

# Defining a PID controller
class PID:
    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt, kp, ki, kd, i_limit):
        # I term - clamped so it can't keep growing forever else it may overreact causing system to osscilate or worse overshoot.
        # past term that accumulates existing error to make sure
        # we get to 0
        self.integral += error * dt
        self.integral = max(-i_limit, min(i_limit, self.integral))

        # D term - how fast the error is changing i.e. the future. Dont' calculate derivative for first step.
        derivative = 0.0
        if self.prev_error is not None and dt > 0.0:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # proportional to error * accumulated error over a certain time frame * how that error is changing
        return kp * error + ki * self.integral + kd * derivative

# clamps the PID output such that the ang/lin output never exceeds the limits defined in the params
def clamp(x, limit):
    return max(-limit, min(limit, x))

def main():
    rclpy.init()
    node = rclpy.create_node('chase_object')
    cmd_vel_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 5)

    for key, value in params.items():
        node.declare_parameter(key, value)

    def get_parameter_value(name):
        return node.get_parameter(name).value

    # Instantiate 2 PID controllers.
    ang_pid = PID()
    lin_pid = PID()
    # Time we last got a reading with the ball in it. None = we're stopped / starting fresh
    state = {'last_seen': None}

    def stop():
        cmd_vel_publisher.publish(Twist())
        ang_pid.reset()
        lin_pid.reset()
        state['last_seen'] = None

    def range_callback(msg):
        if not msg.found:
            if state['last_seen'] is None:
                # Already stopped - make sure we stay stopped
                cmd_vel_publisher.publish(Twist())
            return

        now = node.get_clock().now().nanoseconds * 1e-9
        # Time since the last reading.
        if state['last_seen'] is None:
            dt = 0.0
        else:
            dt = now - state['last_seen']

        state['last_seen'] = now

        # Errors. Ball on the left -> bearing is + -> turn left (+z). Ball too far -> + error -> drive forward.
        ang_error = msg.bearing
        lin_error = msg.distance - get_parameter_value('desired_distance')

        twist = Twist()
        # For angular error we don't care about whether its too the left/right but about its magnitude. If within deadband then stay put
        if abs(ang_error) > get_parameter_value('ang_deadband'):
            w = ang_pid.update(ang_error, dt, get_parameter_value('ang_kp'), get_parameter_value('ang_ki'), get_parameter_value('ang_kd'), get_parameter_value('i_limit'))
            twist.angular.z = clamp(w, get_parameter_value('max_ang_vel'))
        else:
            ang_pid.reset()

        if abs(lin_error) > get_parameter_value('lin_deadband'):
            v = lin_pid.update(lin_error, dt, get_parameter_value('lin_kp'), get_parameter_value('lin_ki'), get_parameter_value('lin_kd'), get_parameter_value('i_limit'))
            twist.linear.x = clamp(v, get_parameter_value('max_lin_vel'))
        else:
            lin_pid.reset()

        # Slow the forward/backward speed down the more the ball is off to the side.
        twist.linear.x *= max(0.0, math.cos(ang_error))

        cmd_vel_publisher.publish(twist)

    def watchdog():
        # If detect_object / get_object_range die or wifi drops, don't keep driving on the last command
        if state['last_seen'] is not None:
            now = node.get_clock().now().nanoseconds * 1e-9
            if now - state['last_seen'] > get_parameter_value('timeout'):
                stop()   # stop() also clears last_seen, so this only fires once

    node.create_subscription(ObjLocation, RANGE_TOPIC, range_callback, CUSTOM_QOS_PROFILE)
    node.create_timer(0.1, watchdog)

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
