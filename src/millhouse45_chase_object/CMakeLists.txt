cmake_minimum_required(VERSION 3.8)
project(millhouse45_chase_object)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)        # <-- ament_cmake_python
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjLocation.msg"
)

# Python nodes (installed where `ros2 run` looks for them)
install(PROGRAMS
  scripts/detector.py
  scripts/detect_object.py
  scripts/get_object_range.py
  scripts/chase_object.py
  scripts/disp_cam.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
