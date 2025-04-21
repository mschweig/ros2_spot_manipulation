cmake_minimum_required(VERSION 3.8)
project(spot_manipulation_interface)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Grasp.action"
  DEPENDENCIES builtin_interfaces
)

# Use 'src' as Python package
ament_python_install_package(src)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/spot_grasp_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spot_grasp_action_server
)

ament_package()