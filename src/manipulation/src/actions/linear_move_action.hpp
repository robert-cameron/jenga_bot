#pragma once

#include "constrained_move_action.hpp"

class LinearMoveAction : public ConstrainedMoveAction
{
public:
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

    geometry_msgs::msg::Pose target_pose = goal.block_pose;
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;       // 1 cm resolution
    const double jump_threshold = 0.0;

    double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.99) {
      feedback->feedback = "Failed to compute full Cartesian path (" + std::to_string(fraction * 100.0) + "% achieved)";
      goal_handle->publish_feedback(feedback);
      move_group.clearPathConstraints();
      move_group.clearPoseTargets();
      return false;
    }

    feedback->feedback = "Cartesian path computed successfully (" + std::to_string(fraction * 100.0) + "%). Executing...";
    goal_handle->publish_feedback(feedback);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    move_group.execute(plan);

    feedback->feedback = "LinearMoveAction completed successfully.";
    goal_handle->publish_feedback(feedback);

    move_group.clearPathConstraints();
    move_group.clearPoseTargets();
    return true;
  }
};
