cmake_minimum_required(VERSION 3.8)
project(gui_node)

# Basic deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(tower_interfaces REQUIRED)

# Qt moc/uic/rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# -------------------------------
# Executable node: gui_node
# -------------------------------
add_executable(gui_node src/gui_node.cpp)
ament_target_dependencies(gui_node
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2
  tower_interfaces
)
install(TARGETS gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# RQT plugin: jenga_rqt_plugin (optional)
# -------------------------------
add_library(jenga_rqt_plugin SHARED src/jenga_rqt_plugin.cpp)
ament_target_dependencies(jenga_rqt_plugin
  rclcpp
  std_msgs
  rqt_gui_cpp
)
target_link_libraries(jenga_rqt_plugin Qt5::Widgets)

# Export plugin description
pluginlib_export_plugin_description_file(rqt_gui_cpp jenga_rqt_plugin.xml)

install(TARGETS jenga_rqt_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Install resources
# -------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
install(FILES launch/jenga_config.rviz
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
