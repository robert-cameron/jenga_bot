#include "os_node_window.hpp"
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

OSNodeWindow::OSNodeWindow(QWidget* parent)
  : QWidget(parent)
{
  QVBoxLayout* main_layout = new QVBoxLayout(this);

  // Buttons
  QHBoxLayout* button_layout = new QHBoxLayout();
  QPushButton* start_btn     = new QPushButton("Start Game");
  QPushButton* stop_btn      = new QPushButton("Stop Game");
  QPushButton* push_btn      = new QPushButton("Try Push");
  QPushButton* calibrate_btn = new QPushButton("Calibrate");
  QPushButton* turn_btn      = new QPushButton("Take Turn");

  button_layout->addWidget(start_btn);
  button_layout->addWidget(stop_btn);
  button_layout->addWidget(push_btn);
  button_layout->addWidget(calibrate_btn);
  button_layout->addWidget(turn_btn);

  main_layout->addLayout(button_layout);

  // Sensor display
  force_label_ = new QLabel("Force Sensor: 0.0 N");
  angle_label_ = new QLabel("Servo Angle: 0.0°");
  main_layout->addWidget(force_label_);
  main_layout->addWidget(angle_label_);

  // Legend
  QGroupBox* legend_box = new QGroupBox("Legend");
  QVBoxLayout* legend_layout = new QVBoxLayout();
  legend_layout->addWidget(new QLabel("Gray = Unknown"));
  legend_layout->addWidget(new QLabel("Green = Pushable"));
  legend_layout->addWidget(new QLabel("Orange = Not Pushable"));
  legend_box->setLayout(legend_layout);
  main_layout->addWidget(legend_box);

  // ROS2 node
  node_ = rclcpp::Node::make_shared("OS_node");
  force_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "force_sensor", 10,
    [this](std_msgs::msg::Float32::SharedPtr msg) {
      force_label_->setText(QString("Force Sensor: %1 N").arg(msg->data, 0, 'f', 2));
    });

  angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "servo_angle", 10,
    [this](std_msgs::msg::Float32::SharedPtr msg) {
      angle_label_->setText(QString("Servo Angle: %1°").arg(msg->data, 0, 'f', 1));
    });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

OSNodeWindow::~OSNodeWindow()
{
  executor_->cancel();
  if (spin_thread_.joinable())
    spin_thread_.join();
  node_.reset();
}
