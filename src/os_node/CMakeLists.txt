cmake_minimum_required(VERSION 3.8)
project(os_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_executable(os_node
  src/main.cpp
  src/os_node_window.cpp
)

ament_target_dependencies(os_node
  rclcpp
  std_msgs
)

target_link_libraries(os_node
  Qt5::Widgets
)

install(TARGETS os_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
