#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tower_interfaces/msg/tower.hpp>
#include <tower_interfaces/msg/tower_row.hpp>

class GuiNode : public rclcpp::Node {
public:
  GuiNode() : Node("gui_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("jenga_markers", 10);

    // 订阅识别节点发布的 PoseArray
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "vision/blocks", 10,
      std::bind(&GuiNode::poseCallback, this, std::placeholders::_1));

    // 订阅识别节点发布的 Tower occupancy
    tower_sub_ = this->create_subscription<tower_interfaces::msg::Tower>(
      "vision/tower", 10,
      std::bind(&GuiNode::towerCallback, this, std::placeholders::_1));
  }

private:
  void towerCallback(const tower_interfaces::msg::Tower::SharedPtr msg) {
    latest_tower_ = *msg;  // 保存最新的 Tower 消息
  }

  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;

    double block_length = 0.13;
    double block_width  = 0.04;
    double block_height = 0.03;

    int marker_id = 0;
    for (const auto &pose : msg->poses) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;  // 使用识别节点的坐标系
      marker.header.stamp = this->now();
      marker.ns = "jenga";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;

      marker.scale.x = block_length;
      marker.scale.y = block_width;
      marker.scale.z = block_height;

      // 默认颜色：灰色
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;

      // 如果 Tower occupancy 有数据，就根据占用情况上色
      if (!latest_tower_.rows.empty()) {
        int level = marker.id / 3;   // 每层 3 个积木
        int pos   = marker.id % 3;   // 该层的第几个积木

        if (level < static_cast<int>(latest_tower_.rows.size())) {
          const auto &row = latest_tower_.rows[level];
          bool occupied = (pos == 0 ? row.pos1 : (pos == 1 ? row.pos2 : row.pos3));

          if (occupied) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;  // 绿色：检测到
          } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;  // 红色：缺失
          }
        }
      }

      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %zu markers", marker_array.markers.size());
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<tower_interfaces::msg::Tower>::SharedPtr tower_sub_;

  tower_interfaces::msg::Tower latest_tower_;  // 保存最新的 Tower occupancy
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuiNode>());
  rclcpp::shutdown();
  return 0;
}

