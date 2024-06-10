cmake_minimum_required(VERSION 3.5)
project(quad_robot_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(BASE_LIBS
  ament_cmake
  geometry_msgs
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  std_msgs
)

# quadruped motion executable
add_executable(quadruped_torotto src/quadruped_torotto.cpp)
ament_target_dependencies(quadruped_torotto
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(quadruped_effort src/quadruped_effort.cpp)
ament_target_dependencies(quadruped_effort
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(quadruped_effort_init src/quadruped_effort_init.cpp)
ament_target_dependencies(quadruped_effort_init
  rclcpp
  rclcpp_action
  control_msgs
)

## Install
install(
  TARGETS
  quadruped_torotto
  quadruped_effort
  quadruped_effort_init
  DESTINATION
  lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

## Install
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()