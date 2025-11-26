cmake_minimum_required(VERSION 3.8)
project(gui_node)

# 基本依赖
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

# 自动处理 Qt MOC/UIC/RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# -------------------------------
# 可执行节点：gui_node
# -------------------------------
add_executable(gui_node src/gui_node.cpp)
ament_target_dependencies(gui_node
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2
)
install(TARGETS gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# RQT 插件：jenga_rqt_plugin
# -------------------------------
add_library(jenga_rqt_plugin SHARED src/jenga_rqt_plugin.cpp)
ament_target_dependencies(jenga_rqt_plugin
  rclcpp
  std_msgs
  rqt_gui_cpp
)
target_link_libraries(jenga_rqt_plugin Qt5::Widgets)

# 导出插件描述文件
pluginlib_export_plugin_description_file(rqt_gui_cpp jenga_rqt_plugin.xml)

install(TARGETS jenga_rqt_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# 安装资源
# -------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
