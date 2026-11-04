#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from turtlebot3_msgs.srv import Sound
from turtlebot3_recovery_lab.srv import DivertRobot


BACKUP_DISTANCE = 0.5   # m. How far behind the robot the recovery waypoint goes


class StudentRecoveryServer(Node):
    def __init__(self):
        super().__init__('student_recovery_server')

        # TurtleBot3 OpenCR buzzer service client
        self.sound_client = self.create_client(
            Sound,
            '/sound'
        )

        # Recovery service called by the C++ Behavior Tree bridge
        self.srv = self.create_service(
            DivertRobot,
            '/divert_robot',
            self.handle_recovery
        )

        # Robot pose in the map frame (map -> base_link, published by AMCL)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            'Recovery Python Server listening on /divert_robot'
        )

        if self.sound_client.service_is_ready():
            self.get_logger().info(
                'TurtleBot3 sound service available'
            )
        else:
            self.get_logger().warn(
                'TurtleBot3 sound service not currently available'
            )

    def beep(self, value=3):
        """Trigger a TurtleBot3 buzzer sound asynchronously."""

        if not self.sound_client.service_is_ready():
            self.get_logger().warn(
                'Cannot beep: /sound service unavailable'
            )
            return

        sound_request = Sound.Request()

        # TurtleBot3 sound commands:
        # 0 = OFF
        # 1 = ON
        # 2 = LOW_BATTERY
        # 3 = ERROR
        # 4 = BUTTON1
        # 5 = BUTTON2
        sound_request.value = value

        try:
            future = self.sound_client.call_async(sound_request)
            future.add_done_callback(self.sound_response_callback)

            self.get_logger().info(
                f'Sent sound request {value} to TurtleBot3'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Failed to send sound request: {exc}'
            )

    def sound_response_callback(self, future):
        """Process the asynchronous sound service response."""

        try:
            result = future.result()

            if result.success:
                self.get_logger().info(
                    'TurtleBot3 buzzer activated successfully'
                )
            else:
                self.get_logger().warn(
                    f'TurtleBot3 buzzer failed: {result.message}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Sound service call failed: {exc}'
            )

    def handle_recovery(self, request, response):
        """
        Called when Nav2 triggers the custom recovery behavior.

        1. Trigger the TurtleBot3 buzzer.
        2. Compute a recovery waypoint.
        3. Return the waypoint to the C++ BT bridge if one is available.
        4. Trigger a different Turtlebot3 buzzer if successful.
        """

        self.get_logger().warn(
            'Nav2 stalled! Sounding alarm and calculating diversion route...'
        )

        # --------------------------------------------------
        # STEP 1: Trigger the TurtleBot3 buzzer
        # --------------------------------------------------

        self.beep(3)    # ERROR

        # --------------------------------------------------
        # STEP 2: Compute recovery waypoint
        # --------------------------------------------------

        # Back up: the waypoint is BACKUP_DISTANCE behind the robot, facing the same way.
        # The path ahead is blocked, so the way we came is the safest place to go.
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as exc:
            self.get_logger().error(f'No robot pose, cannot pick a recovery waypoint: {exc}')
            response.success = False
            return response

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        recovery_x = x - BACKUP_DISTANCE * math.cos(yaw)
        recovery_y = y - BACKUP_DISTANCE * math.sin(yaw)

        response.recovery_pose.header.frame_id = 'map'
        response.recovery_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        response.recovery_pose.pose.position.x = recovery_x
        response.recovery_pose.pose.position.y = recovery_y
        response.recovery_pose.pose.position.z = 0.0

        # Keep the robot's current heading
        response.recovery_pose.pose.orientation = q

        # --------------------------------------------------
        # STEP 3: Return recovery waypoint
        # --------------------------------------------------

        response.success = True

        self.get_logger().info(
            f'Recovery waypoint generated: '
            f'({recovery_x:.2f}, {recovery_y:.2f})'
        )

        # ---------------------------------------------------
        # STEP 4: Trigger a different TurtleBot3 buzzer if a
        # recovery point was found
        # ---------------------------------------------------

        self.beep(1)    # ON

        return response


def main(args=None):
    rclpy.init(args=args)

    node = StudentRecoveryServer()

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
