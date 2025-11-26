#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tower_interfaces/msg/tower.hpp>
#include <tower_interfaces/msg/tower_row.hpp>

class GuiNode : public rclcpp::Node {
public:
  GuiNode() : Node("gui_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("jenga_markers", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "vision/blocks", 10,
      std::bind(&GuiNode::poseCallback, this, std::placeholders::_1));

    tower_sub_ = this->create_subscription<tower_interfaces::msg::Tower>(
      "vision/tower", 10,
      std::bind(&GuiNode::towerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GuiNode started: subscribing to vision/blocks and vision/tower.");
  }

private:
  void towerCallback(const tower_interfaces::msg::Tower::SharedPtr msg) {
    latest_tower_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received Tower: %zu rows", latest_tower_.rows.size());
  }

  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Optional: Clear previous markers to avoid ghosting
    {
      visualization_msgs::msg::Marker delete_all;
      delete_all.header.frame_id = msg->header.frame_id;
      delete_all.header.stamp = this->now();
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(delete_all);
    }

    // Keep these in sync with your detection node
    const double block_length = 0.075; // meters
    const double block_width  = 0.025; // meters
    const double block_height = 0.015; // meters

    int id = 0;
    for (const auto &pose : msg->poses) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;  // e.g., "tower_base"
      marker.header.stamp = this->now();
      marker.ns = "jenga";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;

      marker.scale.x = block_length;
      marker.scale.y = block_width;
      marker.scale.z = block_height;

      // Default color: gray (no Tower info yet)
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;

      // Color by occupancy if available.
      // Assumption: detection node appends poses in level order, 3 blocks per level.
      if (!latest_tower_.rows.empty()) {
        const int level = id / 3;
        const int pos   = id % 3;
        if (level < static_cast<int>(latest_tower_.rows.size())) {
          const auto &row = latest_tower_.rows[level];
          const bool occupied = (pos == 0 ? row.pos1 : (pos == 1 ? row.pos2 : row.pos3));
          if (occupied) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;  // green
          } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;  // red
          }
        }
      }

      marker_array.markers.push_back(marker);
      ++id;
    }

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %zu markers (frame: %s)",
                marker_array.markers.size(), marker_array.markers.empty() ? "n/a" : marker_array.markers[0].header.frame_id.c_str());
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<tower_interfaces::msg::Tower>::SharedPtr tower_sub_;

  tower_interfaces::msg::Tower latest_tower_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuiNode>());
  rclcpp::shutdown();
  return 0;
}
