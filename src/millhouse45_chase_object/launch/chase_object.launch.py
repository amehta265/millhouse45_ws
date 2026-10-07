# Starts the three robot-side nodes in one go (run on the robot, after camera_robot.launch.py)
# This prevents us from running each node separately on individual terminals
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='millhouse45_chase_object', executable='detect_object.py', name='detect_object', output='screen'),
        Node(package='millhouse45_chase_object', executable='get_object_range.py', name='get_object_range', output='screen'),
        Node(package='millhouse45_chase_object', executable='chase_object.py', name='chase_object', output='screen'),
    ])
