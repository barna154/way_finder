cmake_minimum_required(VERSION 3.8)
project(way_finder)

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(megoldas2 src/megoldas2.cpp)
ament_target_dependencies(megoldas2
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs rcl_interfaces
)

install(TARGETS megoldas2
  DESTINATION lib/${PROJECT_NAME}
)