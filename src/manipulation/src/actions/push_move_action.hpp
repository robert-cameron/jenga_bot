#pragma once

#include "base_action.hpp"
#include "constrained_move_action.hpp"
#include "linear_move_action.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>

class PushMoveAction : public BaseAction
{
public:
  explicit PushMoveAction(double push_distance = 0.08, double push_speed = 0.1, double retract_speed = 0.2)
  : constrained_action_(),
    linear_push_action_(push_speed),
    linear_retract_action_(retract_speed),
    push_distance_(push_distance)
  {
  }

  bool execute(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const manipulation::action::Manipulation::Goal& goal,
    std::shared_ptr<GoalHandleManipulation> goal_handle) override
  {
    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing PushMoveAction: moving to pre-push pose...";
    goal_handle->publish_feedback(feedback);

    if (!constrained_action_.execute(move_group, goal, goal_handle))
    {
      feedback->feedback = "PushMoveAction failed during constrained approach.";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    feedback->feedback = "Computing push direction and target pose...";
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::Pose push_pose = goal.block_pose;
    tf2::Quaternion orientation;
    tf2::fromMsg(push_pose.orientation, orientation);
    orientation.normalize();

    tf2::Vector3 forward_vector(0.0, 1.0, 0.0);
    tf2::Vector3 push_vector = tf2::quatRotate(orientation, forward_vector).normalized() * push_distance_;

    push_pose.position.x += push_vector.x();
    push_pose.position.y += push_vector.y();
    push_pose.position.z += push_vector.z();

    manipulation::action::Manipulation::Goal push_goal = goal;
    push_goal.block_pose = push_pose;

    rclcpp::sleep_for(std::chrono::seconds(1));

    // push
    feedback->feedback = "Executing push with linear move...";
    goal_handle->publish_feedback(feedback);
    if (!linear_push_action_.execute(move_group, push_goal, goal_handle))
    {
      feedback->feedback = "PushMoveAction failed during push.";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // retract
    feedback->feedback = "Executing pull-back with linear move...";
    goal_handle->publish_feedback(feedback);
    if (!linear_retract_action_.execute(move_group, goal, goal_handle))
    {
      feedback->feedback = "PushMoveAction failed during retract.";
      goal_handle->publish_feedback(feedback);
      return false;
    }

    feedback->feedback = "PushMoveAction completed successfully.";
    goal_handle->publish_feedback(feedback);
    return true;
  }

private:
  ConstrainedMoveAction constrained_action_;
  LinearMoveAction linear_push_action_;
  LinearMoveAction linear_retract_action_;
  double push_distance_;
};
