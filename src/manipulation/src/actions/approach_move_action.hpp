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
                              geometry_msgs::msg::Pose end_eff_pose, double approach_distance = 0.02, double speed = 0.6)
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
      return true;
    }

    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing ApproachMoveAction...";
    goal_handle->publish_feedback(feedback);

    int curr_face_id = estimateFaceId(tower_tf.transform.rotation, end_eff_pose.orientation);
    int new_face_id = estimateFaceId(tower_tf.transform.rotation, approach_goal.pose.orientation);

    auto corner_poses = getCornerPoses(tower_tf, 1.4142 * (TOWER_WIDTH / 2));

    double height_diff = approach_goal.pose.position.z - end_eff_pose.position.z;
    auto path = getPathBetweenNodes(curr_face_id, new_face_id, end_eff_pose.position.z, approach_goal.pose.position.z,
                                    std::vector<geometry_msgs::msg::Pose>(corner_poses.begin(), corner_poses.end()));

    path.push_back(approach_goal.pose);

    for (auto pose : path)
    {
      manipulation::action::Manipulation::Goal goal;
      goal.pose = pose;
      LinearMoveAction action(node_, speed);

      if (!action.execute(move_group, goal, goal_handle))
      {
        feedback->feedback = "ApproachMoveAction failed.";
        goal_handle->publish_feedback(feedback);
        RCLCPP_ERROR(node_->get_logger(), "[ApproachMoveAction] Move failed at pose (%.3f, %.3f, %.3f)",
                     pose.position.x, pose.position.y, pose.position.z);
        return false;
      }
    }

    return true;
  }

  static manipulation::action::Manipulation::Goal getApproachGoal(manipulation::action::Manipulation::Goal goal, double approach_distance = 0.02)
  {
    manipulation::action::Manipulation::Goal approachGoal = goal;

    tf2::Quaternion q;
    tf2::fromMsg(goal.pose.orientation, q);

    tf2::Vector3 backwards_vector(-1.0, 0.0, 0.0);
    tf2::Vector3 adjustment = tf2::quatRotate(q, backwards_vector).normalized() * approach_distance;

    approachGoal.pose.position.x += adjustment.x();
    approachGoal.pose.position.y += adjustment.y();
    approachGoal.pose.position.z += adjustment.z();

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

  int estimateFaceId(const geometry_msgs::msg::Quaternion &q1, const geometry_msgs::msg::Quaternion &q2)
  {
    tf2::Quaternion tf_q1(q1.x, q1.y, q1.z, q1.w);
    tf2::Quaternion tf_q2(q2.x, q2.y, q2.z, q2.w);
    double r1, p1, y1;
    double r2, p2, y2;
    tf2::Matrix3x3(tf_q1).getRPY(r1, p1, y1);
    tf2::Matrix3x3(tf_q2).getRPY(r2, p2, y2);
    double dyaw = (y2 - y1) * 180.0 / M_PI;
    while (dyaw > 180.0)
      dyaw -= 360.0;
    while (dyaw <= -180.0)
      dyaw += 360.0;
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
    return face_id;
  }

  std::vector<geometry_msgs::msg::Pose> getPathBetweenNodes(
      int startNode,
      int endNode,
      double startHeight,
      double endHeight,
      const std::vector<geometry_msgs::msg::Pose> &corners)
  {

    double heightDiff = endHeight - startHeight;
    std::vector<geometry_msgs::msg::Pose> path;

    for (int i = std::min(startNode, endNode); i < std::max(startNode, endNode); i++)
    {
      path.push_back(corners[i]);
    }

    if (startNode > endNode)
    {
      std::reverse(path.begin(), path.end());
    }

    // update heights of corners
    for (int i = 0; i < path.size(); i++)
    {
      path[i].position.z = startHeight + (i + 1) * heightDiff / (path.size());
    }

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
        {-d, -d, 0.0}};

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
    }

    return poses;
  }

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::Pose end_eff_pose;
  double approach_distance;
  double speed;
  geometry_msgs::msg::TransformStamped tower_tf;
};
