cmake_minimum_required(VERSION 3.8)
project(transport_system_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPosition.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()
