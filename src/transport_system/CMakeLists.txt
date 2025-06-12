cmake_minimum_required(VERSION 3.8)
project(transport_system)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Ajout de l'action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPosition.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY transport_system
  DESTINATION lib/${PYTHON_INSTALL_DIR}
)

ament_package()
