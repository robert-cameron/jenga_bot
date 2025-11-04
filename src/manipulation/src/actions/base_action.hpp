#pragma once
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "manipulation/action/manipulation.hpp"

using GoalHandleManipulation = rclcpp_action::ServerGoalHandle<manipulation::action::Manipulation>;

class BaseAction {
public:
  virtual ~BaseAction() = default;

  virtual bool execute(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const manipulation::action::Manipulation::Goal& goal,
    std::shared_ptr<GoalHandleManipulation> goal_handle
  ) = 0;
};
