#include <QApplication>
#include "os_node_window.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  OSNodeWindow window;
  window.show();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
