#pragma once

#define TOWER_WIDTH 0.13

#include "base_action.hpp"
#include "constrained_move_action.hpp"
#include "linear_move_action.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm> 

class ApproachMoveAction : public BaseAction
{
public:
  explicit ApproachMoveAction(rclcpp::Node::SharedPtr node,
                              geometry_msgs::msg::Pose end_eff_pose, double approach_distance = 0.02, double speed = 0.5)
      : node_(node),
        end_eff_pose(end_eff_pose),
        approach_distance(approach_distance),
        speed(speed)
  {
    auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (!tf_buffer_->canTransform("base_link", "tower_base", tf2::TimePointZero, tf2::durationFromSec(2.0)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for transform base_link -> tower_base");
    }

    try
    {
      tower_tf = tf_buffer_->lookupTransform("base_link", "tower_base", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Transform lookup failed even after waiting: %s", ex.what());
    }
  }

  bool execute(
      moveit::planning_interface::MoveGroupInterface &move_group,
      const manipulation::action::Manipulation::Goal &goal,
      std::shared_ptr<GoalHandleManipulation> goal_handle) override
  {

    const manipulation::action::Manipulation::Goal approach_goal = getApproachGoal(goal);
    std::vector<manipulation::action::Manipulation::Goal> goals;

    // check if already at the desired pose
    if (posesEqual(approach_goal.pose, end_eff_pose))
    {
      auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
      feedback->feedback = "Skipping ApproachMoveAction - already at goal pose";
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(node_->get_logger(), "[ApproachMoveAction] Already at target pose, skipping execution.");
      return true;
    }

    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing ApproachMoveAction...";
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(node_->get_logger(), "[ApproachMoveAction] Starting execution...");
    RCLCPP_INFO(node_->get_logger(), "End effector goal: (%.3f, %.3f, %.3f)",
                end_eff_pose.position.x, end_eff_pose.position.y, end_eff_pose.position.z);

    int curr_face_id = estimateFaceId(tower_tf.transform.rotation, end_eff_pose.orientation);
    int new_face_id = estimateFaceId(approach_goal.pose.orientation, end_eff_pose.orientation);

    RCLCPP_INFO(node_->get_logger(), "[Pathfinding] Current face ID: %d, Target face ID: %d", curr_face_id, new_face_id);

    auto corner_poses = getCornerPoses(tower_tf, 0.7071 * (TOWER_WIDTH + 2 * approach_distance));
    RCLCPP_INFO(node_->get_logger(), "[Pathfinding] Generated corner poses for tower navigation.");

    auto path = getPathBetweenNodes(curr_face_id, new_face_id,
                                    std::vector<geometry_msgs::msg::Pose>(corner_poses.begin(), corner_poses.end()));

    RCLCPP_INFO(node_->get_logger(), "[Pathfinding] Computed path with %zu intermediate nodes.", path.size());
    for (size_t i = 0; i < path.size(); ++i)
    {
      RCLCPP_INFO(node_->get_logger(), "[Pathfinding] Path point %zu: (%.3f, %.3f, %.3f)",
                  i, path[i].position.x, path[i].position.y, path[i].position.z);
    }

    path.push_back(approach_goal.pose);
    RCLCPP_INFO(node_->get_logger(), "[Pathfinding] Added approach goal to path. Total waypoints: %zu", path.size());

    for (auto pose : path)
    {
      manipulation::action::Manipulation::Goal goal;
      goal.pose = pose;
      LinearMoveAction action(speed);

      RCLCPP_INFO(node_->get_logger(), "[Pathfinding] Executing move to (%.3f, %.3f, %.3f)",
                  pose.position.x, pose.position.y, pose.position.z);

      if (!action.execute(move_group, goal, goal_handle))
      {
        feedback->feedback = "ApproachMoveAction failed.";
        goal_handle->publish_feedback(feedback);
        RCLCPP_ERROR(node_->get_logger(), "[ApproachMoveAction] Move failed at pose (%.3f, %.3f, %.3f)",
                     pose.position.x, pose.position.y, pose.position.z);
        return false;
      }
    }

    RCLCPP_INFO(node_->get_logger(), "[ApproachMoveAction] Completed path successfully.");
    return true;
  }

  manipulation::action::Manipulation::Goal getApproachGoal(manipulation::action::Manipulation::Goal goal)
  {
    manipulation::action::Manipulation::Goal approachGoal = goal;

    tf2::Quaternion q;
    tf2::fromMsg(goal.pose.orientation, q);

    tf2::Vector3 backwards_vector(-1.0, 0.0, 0.0);
    tf2::Vector3 adjustment = tf2::quatRotate(q, backwards_vector).normalized() * approach_distance;

    approachGoal.pose.position.x += adjustment.x();
    approachGoal.pose.position.y += adjustment.y();
    approachGoal.pose.position.z += adjustment.z();

    RCLCPP_INFO(node_->get_logger(),
                "[ApproachGoal] Adjusted pose by (%.3f, %.3f, %.3f) for approach distance %.3f",
                adjustment.x(), adjustment.y(), adjustment.z(), approach_distance);

    return approachGoal;
  }

private:
  bool posesEqual(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2, double pos_tolerance = 1e-3, double ang_tolerance = 1e-2)
  {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    double dz = p1.position.z - p2.position.z;
    double pos_diff = std::sqrt(dx * dx + dy * dy + dz * dz);

    tf2::Quaternion q1, q2;
    tf2::fromMsg(p1.orientation, q1);
    tf2::fromMsg(p2.orientation, q2);

    double angle_diff = q1.angleShortestPath(q2);

    return (pos_diff < pos_tolerance) && (angle_diff < ang_tolerance);
  }

  int estimateFaceId(const geometry_msgs::msg::Quaternion &q1,
                     const geometry_msgs::msg::Quaternion &q2)
  {
    tf2::Quaternion tf_q1(q1.x, q1.y, q1.z, q1.w);
    tf2::Quaternion tf_q2(q2.x, q2.y, q2.z, q2.w);

    double r1, p1, y1;
    double r2, p2, y2;

    tf2::Matrix3x3(tf_q1).getRPY(r1, p1, y1);
    tf2::Matrix3x3(tf_q2).getRPY(r2, p2, y2);

    double dyaw = (y2 - y1) * 180.0 / M_PI;

    while (dyaw > 180.0) dyaw -= 360.0;
    while (dyaw <= -180.0) dyaw += 360.0;

    RCLCPP_INFO(node_->get_logger(), "[estimateFaceId] dyaw = %.2f degrees", dyaw);

    const double TOL = 15.0;

    int face_id;
    if (std::fabs(dyaw) < TOL)
      face_id = 2;
    else if (std::fabs(dyaw - 90.0) < TOL)
      face_id = 3;
    else if (std::fabs(dyaw + 90.0) < TOL)
      face_id = 1;
    else if (std::fabs(std::fabs(dyaw) - 180.0) < TOL)
      face_id = 0;
    else
      face_id = -1;

    RCLCPP_INFO(node_->get_logger(), "[estimateFaceId] Face ID determined: %d", face_id);
    return face_id;
  }

  std::vector<geometry_msgs::msg::Pose> getPathBetweenNodes(
      int startNode,
      int endNode,
      const std::vector<geometry_msgs::msg::Pose> &edges)
  {
    std::vector<geometry_msgs::msg::Pose> path;

    RCLCPP_INFO(node_->get_logger(), "[getPathBetweenNodes] Building path from node %d to %d", startNode, endNode);

    for (int i = std::min(startNode, endNode); i < std::max(startNode, endNode); i++) {
      path.push_back(edges[i]);
      RCLCPP_INFO(node_->get_logger(), "[getPathBetweenNodes] Added edge index %d to path", i);
    }

    if (startNode > endNode) {
      std::reverse(path.begin(), path.end());
      RCLCPP_INFO(node_->get_logger(), "[getPathBetweenNodes] Path reversed (start > end).");
    }

    RCLCPP_INFO(node_->get_logger(), "[getPathBetweenNodes] Final path length: %zu", path.size());
    return path;
  }

  std::array<geometry_msgs::msg::Pose, 3> getCornerPoses(
      const geometry_msgs::msg::TransformStamped &tower_tf, double d)
  {
    std::array<geometry_msgs::msg::Pose, 3> poses;

    tf2::Transform tower_transform;
    tf2::fromMsg(tower_tf.transform, tower_transform);

    std::vector<tf2::Vector3> offsets = {
        {d, d, 0.0},
        {-d, d, 0.0},
        {-d, -d, 0.0}
    };

    for (size_t i = 0; i < offsets.size(); ++i)
    {
      tf2::Vector3 corner_world = tower_transform * offsets[i];
      double yaw = std::atan2(-offsets[i].y(), -offsets[i].x());
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);

      tf2::Quaternion tower_q;
      tf2::fromMsg(tower_tf.transform.rotation, tower_q);
      tf2::Quaternion world_q = tower_q * q;

      geometry_msgs::msg::Pose pose;
      pose.position.x = corner_world.x();
      pose.position.y = corner_world.y();
      pose.position.z = corner_world.z();
      pose.orientation = tf2::toMsg(world_q);

      poses[i] = pose;
      RCLCPP_INFO(node_->get_logger(),
                  "[getCornerPoses] Corner %zu: (%.3f, %.3f, %.3f), yaw = %.2f deg",
                  i, pose.position.x, pose.position.y, pose.position.z, yaw * 180.0 / M_PI);
    }

    return poses;
  }

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::Pose end_eff_pose;
  double approach_distance;
  double speed;
  geometry_msgs::msg::TransformStamped tower_tf;
};
