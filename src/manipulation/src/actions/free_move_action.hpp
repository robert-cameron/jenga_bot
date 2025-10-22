#pragma once

#include "base_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>

class FreeMoveAction : public BaseAction
{
public:
  bool execute(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const manipulation::action::Manipulation::Goal& goal,
    std::shared_ptr<GoalHandleManipulation> goal_handle) override
  {
    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing FreeMoveAction...";
    goal_handle->publish_feedback(feedback);

    move_group.clearPathConstraints();
    move_group.setStartStateToCurrentState();

    move_group.setPoseTarget(goal.block_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      feedback->feedback = "Plan found, executing trajectory...";
      goal_handle->publish_feedback(feedback);

      move_group.execute(plan);

      feedback->feedback = "FreeMoveAction completed successfully.";
      goal_handle->publish_feedback(feedback);
      return true;
    } else {
      feedback->feedback = "FreeMoveAction planning failed.";
      goal_handle->publish_feedback(feedback);
      return false;
    }
  }
};
