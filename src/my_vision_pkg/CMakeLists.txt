cmake_minimum_required(VERSION 3.15)
project(my_vision_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Circle.msg"
  "msg/Circles.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
