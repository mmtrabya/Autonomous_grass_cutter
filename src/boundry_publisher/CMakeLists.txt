cmake_minimum_required(VERSION 3.5)
project(boundary_publisher)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# declare the executable
add_executable(boundary_publisher src/boundary_publisher.cpp)

# link dependencies
ament_target_dependencies(boundary_publisher
  rclcpp
  geometry_msgs
)

# install the executable
install(
  TARGETS boundary_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

