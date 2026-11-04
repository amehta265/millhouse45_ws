cmake_minimum_required(VERSION 3.8)
project(millhouse45_navigate_to_goal)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)        # <-- ament_cmake_python
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/Obstacle.msg"
)

# Python nodes (installed where `ros2 run` looks for them).
# nav_utils.py is not a node, but it has to sit next to them so `from nav_utils import ...` works.
install(PROGRAMS
  scripts/nav_utils.py
  scripts/fixed_odom.py
  scripts/get_object_range.py
  scripts/go_to_goal.py
  scripts/avoid_obstacle.py
  scripts/follow_wall.py
  scripts/behavior_manager.py
  scripts/lookahead_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
