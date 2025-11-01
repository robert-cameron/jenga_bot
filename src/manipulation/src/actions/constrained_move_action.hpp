#pragma once

#include "base_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class ConstrainedMoveAction : public BaseAction
{
public:
  ConstrainedMoveAction()
  {
    setupDefaultConstraints();
  }

  bool execute(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const manipulation::action::Manipulation::Goal& goal,
    std::shared_ptr<GoalHandleManipulation> goal_handle) override
  {
    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing ConstrainedMoveAction...";
    goal_handle->publish_feedback(feedback);

    move_group.setPathConstraints(constraints_);
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal.block_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      feedback->feedback = "Plan found. Executing...";
      goal_handle->publish_feedback(feedback);

      move_group.execute(plan);

      feedback->feedback = "ConstrainedMoveAction completed successfully.";
      goal_handle->publish_feedback(feedback);
      move_group.clearPathConstraints();
      move_group.clearPoseTargets();
      return true;
    }
    else
    {
      feedback->feedback = "Planning failed.";
      goal_handle->publish_feedback(feedback);
      move_group.clearPathConstraints();
      move_group.clearPoseTargets();
      return false;
    }
  }

protected:
  moveit_msgs::msg::Constraints constraints_;

  void setupDefaultConstraints()
  {
    constraints_ = moveit_msgs::msg::Constraints();

    constraints_.joint_constraints.push_back(
      createJointConstraint("shoulder_pan_joint", 0.785398, 1.0472, 1.0472, 1.0));
    constraints_.joint_constraints.push_back(
      createJointConstraint("shoulder_lift_joint", -1.5708, 1.0472, 1.0472, 1.0));
    constraints_.joint_constraints.push_back(
      createJointConstraint("elbow_joint", 1.5708, 1.0472, 1.0472, 1.0));
    constraints_.joint_constraints.push_back(
      createJointConstraint("wrist_1_joint", -1.5708, 1.0472, 1.0472, 1.0));
    constraints_.joint_constraints.push_back(
      createJointConstraint("wrist_2_joint", -1.5708, 0.3, 0.3, 1.0));

    //constraints_.orientation_constraints.push_back(setOrientationDownConstraint().orientation_constraints[0]);
  }

  moveit_msgs::msg::JointConstraint createJointConstraint(
      const std::string &joint_name,
      double position,
      double tolerance_above,
      double tolerance_below,
      double weight)
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = joint_name;
    jc.position = position;
    jc.tolerance_above = tolerance_above;
    jc.tolerance_below = tolerance_below;
    jc.weight = weight;
    return jc;
  }

  moveit_msgs::msg::Constraints setOrientationDownConstraint()
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::OrientationConstraint ocm;

    ocm.link_name = "tool0";
    ocm.header.frame_id = "base_link";
    ocm.absolute_x_axis_tolerance = 0.3;
    ocm.absolute_y_axis_tolerance = 0.3;
    ocm.absolute_z_axis_tolerance = 0.3;
    ocm.weight = 1.0;

    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    ocm.orientation = tf2::toMsg(q);

    constraints.orientation_constraints.push_back(ocm);
    return constraints;
  }
};
