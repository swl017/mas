cmake_minimum_required(VERSION 3.8)
project(mas_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TriangulatedPoint.msg"
  "msg/TriangulatedPointArray.msg"
  "msg/TargetRay.msg"
  "msg/TargetRayArray.msg"
  "msg/TargetRayArrayWithCov.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_package()
