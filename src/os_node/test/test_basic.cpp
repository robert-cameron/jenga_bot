#include <gtest/gtest.h>
#include "os_node_window.hpp"

TEST(OSNodeWindowTest, BasicInit)
{
  int argc = 0;
  char** argv = nullptr;
  QApplication app(argc, argv);

  OSNodeWindow window;
  EXPECT_NE(&window, nullptr);
}
