#pragma once

#include <QWidget>
#include <QLabel>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>

class OSNodeWindow : public QWidget
{
  Q_OBJECT

public:
  explicit OSNodeWindow(QWidget* parent = nullptr);
  ~OSNodeWindow();

private:
  QLabel* force_label_;
  QLabel* angle_label_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread spin_thread_;
};
