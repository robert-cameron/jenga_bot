#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <cstdlib>  // for rand()

struct BlockInfo {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class GuiNode : public rclcpp::Node {
public:
  GuiNode() : Node("gui_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("jenga_markers", 10);

    // 每 1 秒发布一次 MarkerArray
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GuiNode::publishMarkers, this));
  }

private:
  void publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;

    double block_length = 0.13;
    double block_width  = 0.04;
    double block_height = 0.03;

    std::vector<std::vector<BlockInfo>> layer_infos = {
      { {-0.0465, 0.0, 0.015, 0.0, 0.0, M_PI/2}, {0.0, 0.0, 0.015, 0.0, 0.0, M_PI/2}, {0.047, 0.0, 0.015, 0.0, 0.0, M_PI/2} },
      { {0.0, -0.047, 0.045, 0.0, 0.0, M_PI/2}, {0.0, 0.0, 0.045, 0.0, 0.0, M_PI/2}, {0.0, 0.0465, 0.045, 0.0, 0.0, M_PI/2} },
      { {-0.0465, 0.0, 0.075, 0.0, 0.0, M_PI/2}, {0.0, 0.0, 0.075, 0.0, 0.0, M_PI/2}, {0.047, 0.0, 0.075, 0.0, 0.0, M_PI/2} },
      { {0.0, -0.047, 0.105, 0.0, 0.0, M_PI/2}, {0.0, 0.0, 0.105, 0.0, 0.0, M_PI/2}, {0.0, 0.0465, 0.105, 0.0, 0.0, M_PI/2} },
      { {-0.0465, 0.0, 0.135, 0.0, 0.0, M_PI/2}, {0.0, 0.0, 0.135, 0.0, 0.0, M_PI/2}, {0.047, 0.0, 0.135, 0.0, 0.0, M_PI/2} },
    };

    std::vector<int> is_ok_push_array;
    for (int i = 0; i < 15; ++i) {
      is_ok_push_array.push_back(rand() % 3 + 1);  // 1=灰色, 2=绿色, 3=橘色
    }

    for (int layer = 0; layer < static_cast<int>(layer_infos.size()); ++layer) {
      for (int i = 0; i < static_cast<int>(layer_infos[layer].size()); ++i) {
        const BlockInfo& info = layer_infos[layer][i];
        int index = layer * 3 + i;

        geometry_msgs::msg::Pose pose;
        pose.position.x = info.x;
        pose.position.y = info.y;
        pose.position.z = info.z;

        tf2::Quaternion q;
        q.setRPY(info.roll, info.pitch, info.yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        bool horizontal = (layer % 2 == 0);
        double sx = horizontal ? block_length : block_width;
        double sy = horizontal ? block_width : block_length;
        double sz = block_height;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "jenga";
        marker.id = index;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = sx;
        marker.scale.y = sy;
        marker.scale.z = sz;

        int status = is_ok_push_array[index];
        if (status == 1) {
          marker.color.r = 0.5;
          marker.color.g = 0.5;
          marker.color.b = 0.5;
        } else if (status == 2) {
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
        } else if (status == 3) {
          marker.color.r = 1.0;
          marker.color.g = 0.5;
          marker.color.b = 0.0;
        }
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
      }
    }

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %zu markers", marker_array.markers.size());
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuiNode>());
  rclcpp::shutdown();
  return 0;
}
