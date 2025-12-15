cmake_minimum_required(VERSION 3.8)
project(way_finder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(megoldas2 way_finder/megoldas2.cpp)

ament_target_dependencies(megoldas2
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS megoldas2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()