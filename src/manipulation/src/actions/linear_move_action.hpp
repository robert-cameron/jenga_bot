#pragma once

#include "constrained_move_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <functional>

class SafetyStopWatcher
{
public:
  SafetyStopWatcher(rclcpp::Node::SharedPtr node, std::function<void()> stop_callback)
  {
    if (node)
    {
      subscription_ = node->create_subscription<std_msgs::msg::Bool>(
        "/safety/stop", 10,
        [this, stop_callback](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data)
          {
            stop_callback();
          }
        });
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

class LinearMoveAction : public ConstrainedMoveAction
{
public:
  explicit LinearMoveAction(rclcpp::Node::SharedPtr node, double speed_scale = 0.6)
  : node_(node),
    speed_scale_(speed_scale)
  {
  }

  bool execute(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const manipulation::action::Manipulation::Goal& goal,
    std::shared_ptr<GoalHandleManipulation> goal_handle) override
  {
    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing LinearMoveAction (Cartesian)...";
    goal_handle->publish_feedback(feedback);

    move_group.setPathConstraints(constraints_);
    move_group.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = goal.pose;

    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;       // 1 cm resolution
    const double jump_threshold = 0.0;

    double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory, false);

    if (fraction < 0.99)
    {
      feedback->feedback = "Failed to compute full Cartesian path (" + std::to_string(fraction * 100.0) + "% achieved)";
      goal_handle->publish_feedback(feedback);
      move_group.clearPathConstraints();
      move_group.clearPoseTargets();
      return false;
    }

    feedback->feedback = "Cartesian path computed successfully (" + std::to_string(fraction * 100.0) + "%). Executing...";
    goal_handle->publish_feedback(feedback);

    // Scale timing, velocity, acceleration based on speed_scale_
    for (auto &point : trajectory.joint_trajectory.points)
    {
      for (auto &v : point.velocities) v *= speed_scale_;
      for (auto &a : point.accelerations) a *= speed_scale_ * speed_scale_;

      double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      t /= speed_scale_;  // slower = longer
      point.time_from_start.sec = static_cast<int32_t>(floor(t));
      point.time_from_start.nanosec = static_cast<uint32_t>((t - floor(t)) * 1e9);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    std::atomic<bool> safety_stop_triggered{false};
    SafetyStopWatcher watcher(node_, [&]() {
      safety_stop_triggered.store(true);
      move_group.stop();
    });

    move_group.execute(plan);

    if (safety_stop_triggered.load())
    {
      feedback->feedback = "LinearMoveAction interrupted due to safety stop.";
      goal_handle->publish_feedback(feedback);
      move_group.clearPathConstraints();
      move_group.clearPoseTargets();
      return false;
    }

    feedback->feedback = "LinearMoveAction completed successfully.";
    goal_handle->publish_feedback(feedback);

    move_group.clearPathConstraints();
    move_group.clearPoseTargets();
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  double speed_scale_;
};
