cmake_minimum_required(VERSION 3.8)
project(cashier_system)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message and service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bill.msg"
  "srv/GetStatus.srv"
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Create executable
add_executable(bill_generator src/bill_generator.cpp)

# Link dependencies
ament_target_dependencies(bill_generator rclcpp std_msgs)

# Link generated message/service to node
rosidl_target_interfaces(bill_generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executable
install(TARGETS
  bill_generator
  
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(inventory_manager src/inventory_manager.cpp)

ament_target_dependencies(inventory_manager rclcpp std_msgs)

rosidl_target_interfaces(inventory_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  inventory_manager
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(status_viewer src/status_viewer.cpp)

ament_target_dependencies(status_viewer rclcpp std_msgs)

rosidl_target_interfaces(status_viewer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  status_viewer
  DESTINATION lib/${PROJECT_NAME}
)
# Final
ament_package()
