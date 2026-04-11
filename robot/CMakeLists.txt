cmake_minimum_required(VERSION 3.8)
project(robot)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# The global_planner executable
add_executable(global_planner
  src/global_planner_node.cpp
)

# Tell the compiler where to find our header files
target_include_directories(global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link all required ROS2 packages
ament_target_dependencies(global_planner
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

# Install the binary so ros2 run can find it
install(TARGETS global_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install world/urdf/launch folders
install(
  DIRECTORY urdf world launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()