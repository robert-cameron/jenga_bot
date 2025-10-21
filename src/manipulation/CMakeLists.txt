cmake_minimum_required(VERSION 3.8)
project(manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------------------------------------------
# Generate action interface
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Manipulation.action"
  DEPENDENCIES geometry_msgs
)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------
add_executable(manipulation_node src/manipulation.cpp)

ament_target_dependencies(manipulation_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
)

# Link generated action types
rosidl_target_interfaces(manipulation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
install(TARGETS manipulation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
