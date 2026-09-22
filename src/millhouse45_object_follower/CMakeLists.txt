cmake_minimum_required(VERSION 3.8)
project(millhouse45_object_follower)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)        # <-- ament_cmake_python
find_package(rosidl_default_generators REQUIRED)

# Custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjInfo.msg"
)

# Python nodes (installed where `ros2 run` looks for them)
install(PROGRAMS
  scripts/find_object.py
  scripts/rotate_robot.py
  scripts/disp_cam.py
  scripts/detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()