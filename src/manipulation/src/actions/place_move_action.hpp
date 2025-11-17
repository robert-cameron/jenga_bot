#pragma once

#include "base_action.hpp"
#include "approach_move_action.hpp"
#include "linear_move_action.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include "std_msgs/msg/string.hpp"

class PlaceMoveAction : public BaseAction
{
public:
    explicit PlaceMoveAction(rclcpp::Node::SharedPtr node,
                             geometry_msgs::msg::Pose end_eff_pose,
                             double block_offset = 0.056,
                             double clearance_height = 0.04,
                             double drop_height = 0.01,
                             double move_speed = 0.2)
        : node_(node),
          gripper_pub_(node_->create_publisher<std_msgs::msg::String>("/prongs/cmd", 10)),
          end_eff_pose_(end_eff_pose),
          linear_raise_action_(move_speed),
          linear_position_action_(move_speed),
          linear_drop_action_(move_speed),
          linear_reraise_action_(move_speed),
          linear_reset_action_(),
          block_offset_(block_offset),
          clearance_height_(clearance_height),
          drop_height_(drop_height)
    {
    }

    bool execute(
        moveit::planning_interface::MoveGroupInterface &move_group,
        const manipulation::action::Manipulation::Goal &goal,
        std::shared_ptr<GoalHandleManipulation> goal_handle) override
    {
        auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
        feedback->feedback = "Executing PlaceMoveAction: moving up...";
        goal_handle->publish_feedback(feedback);

        // go to clearance position
        auto clearance_goal = goal;
        // force x and y to be above current end eff pos
        clearance_goal.pose.position.x = end_eff_pose_.position.x;
        clearance_goal.pose.position.y = end_eff_pose_.position.y;
        clearance_goal.pose.position.z += clearance_height_; // z = goal height + clearance
        clearance_goal.pose.orientation = end_eff_pose_.orientation;

        if (!linear_raise_action_.execute(move_group, clearance_goal, goal_handle))
        {
            feedback->feedback = "PlaceMoveAction failed during raise.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        // go to drop position
        auto position_goal = ApproachMoveAction::getApproachGoal(goal, -1.0 * block_offset_);
        position_goal.pose.position.z += clearance_height_; // z = goal height + clearance

        if (!linear_position_action_.execute(move_group, position_goal, goal_handle))
        {
            feedback->feedback = "PlaceMoveAction failed during positon.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        // go to drop height
        auto drop_goal = ApproachMoveAction::getApproachGoal(goal, -1.0 * block_offset_);
        drop_goal.pose.position.z += drop_height_; // z = goal height + drop height

        if (!linear_drop_action_.execute(move_group, drop_goal, goal_handle))
        {
            feedback->feedback = "PlaceMoveAction failed during drop.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        // open gripper
        std_msgs::msg::String cmd;
        cmd.data = "o";
        gripper_pub_->publish(cmd);

        rclcpp::sleep_for(std::chrono::seconds(2));

        // raise back up
        if (!linear_reraise_action_.execute(move_group, position_goal, goal_handle))
        {
            feedback->feedback = "PlaceMoveAction failed during rereaise.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        // close gripper
        cmd.data = "cf";
        gripper_pub_->publish(cmd);

        // reset back to tower face
        if (!linear_reset_action_.execute(move_group, clearance_goal, goal_handle))
        {
            feedback->feedback = "PlaceMoveAction failed during reset.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        feedback->feedback = "PlaceMoveAction completed successfully.";
        goal_handle->publish_feedback(feedback);
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> gripper_pub_;
    geometry_msgs::msg::Pose end_eff_pose_;
    LinearMoveAction linear_raise_action_;
    LinearMoveAction linear_position_action_;
    LinearMoveAction linear_drop_action_;
    LinearMoveAction linear_reraise_action_;
    LinearMoveAction linear_reset_action_;
    double block_offset_;
    double clearance_height_;
    double drop_height_;
};
