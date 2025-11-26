#include <rqt_gui_cpp/plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>

namespace gui_node
{

class JengaRqtPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  JengaRqtPlugin()
    : rqt_gui_cpp::Plugin()
  {
    setObjectName("JengaRqtPlugin");
  }

  void initPlugin(qt_gui_cpp::PluginContext& context) override
  {
    widget_ = new QWidget();
    QVBoxLayout* main_layout = new QVBoxLayout(widget_);

    // Buttons
    QHBoxLayout* button_layout = new QHBoxLayout();
    QPushButton* start_btn = new QPushButton("Start Game");
    QPushButton* stop_btn  = new QPushButton("Stop Game");
    QPushButton* push_btn  = new QPushButton("Try Push");
    button_layout->addWidget(start_btn);
    button_layout->addWidget(stop_btn);
    button_layout->addWidget(push_btn);
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

    context.addWidget(widget_);

    // ROS2 node
    node_ = rclcpp::Node::make_shared("jenga_rqt_plugin");
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

    // Spin node in background thread
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void shutdownPlugin() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
      spin_thread_.join();
    node_.reset();
  }

private:
  QWidget* widget_;
  QLabel* force_label_;
  QLabel* angle_label_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread spin_thread_;
};

} // namespace gui_node

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gui_node::JengaRqtPlugin, rqt_gui_cpp::Plugin)
#include "jenga_rqt_plugin.moc"
