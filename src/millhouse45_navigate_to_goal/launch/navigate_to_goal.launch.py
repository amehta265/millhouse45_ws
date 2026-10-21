# Starts all Lab 4 nodes in one go. Run on the robot, after the TurtleBot3 bringup:
#   ros2 launch millhouse45_navigate_to_goal navigate_to_goal.launch.py
# Everything runs on the robot so a WiFi drop cannot leave the robot driving on stale data (handout warning).
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'millhouse45_navigate_to_goal'


def generate_launch_description():
    return LaunchDescription([
        # sensing
        Node(package=PKG, executable='fixed_odom.py', name='fixed_odom', output='screen'),
        Node(package=PKG, executable='get_object_range.py', name='get_object_range', output='screen'),
        # behaviors (each one outputs a velocity vector u)
        Node(package=PKG, executable='go_to_goal.py', name='go_to_goal', output='screen'),
        Node(package=PKG, executable='avoid_obstacle.py', name='avoid_obstacle', output='screen'),
        Node(package=PKG, executable='follow_wall.py', name='follow_wall', output='screen'),
        # decision (state machine) and low level (u -> cmd_vel)
        Node(package=PKG, executable='behavior_manager.py', name='behavior_manager', output='screen'),
        Node(package=PKG, executable='lookahead_controller.py', name='lookahead_controller', output='screen'),
    ])
