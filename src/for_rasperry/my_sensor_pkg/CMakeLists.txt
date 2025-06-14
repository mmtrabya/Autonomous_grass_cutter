cmake_minimum_required(VERSION 3.8)
project(my_sensor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks until licenses are added
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Executable
add_executable(imu src/imu_node.cpp)
add_executable(encoder src/encoder_node.cpp)
# Link libraries and add include dirs
ament_target_dependencies(
  imu
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)
ament_target_dependencies(
  encoder
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)
target_link_libraries(imu wiringPi)
target_link_libraries(encoder wiringPi)

# Install
install(
  TARGETS 
  imu
  encoder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
