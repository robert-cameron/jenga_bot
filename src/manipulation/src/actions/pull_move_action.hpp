#pragma once

#include "base_action.hpp"
#include "approach_move_action.hpp"
#include "linear_move_action.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include "std_msgs/msg/string.hpp"

class PullMoveAction : public BaseAction
{
public:
    explicit PullMoveAction(rclcpp::Node::SharedPtr node,
                            geometry_msgs::msg::Pose end_eff_pose,
                            double pull_distance = 0.08,
                            double drop_height = 0.04,
                            double pull_speed = 0.1)
        : node_(node),
          gripper_pub_(node_->create_publisher<std_msgs::msg::String>("/prongs/cmd", 10)),
          approach_action_(node, end_eff_pose),
          linear_drop_action_(),
          linear_pull_action_(pull_speed),
          drop_height_(drop_height),
          pull_distance_(pull_distance)
    {
    }

    bool execute(
        moveit::planning_interface::MoveGroupInterface &move_group,
        const manipulation::action::Manipulation::Goal &goal,
        std::shared_ptr<GoalHandleManipulation> goal_handle) override
    {
        auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
        feedback->feedback = "Executing PullMoveAction: moving to pre-pull pose...";
        goal_handle->publish_feedback(feedback);

        auto approach_goal = goal;
        approach_goal.pose.position.z += drop_height_;

        if (!approach_action_.execute(move_group, approach_goal, goal_handle))
        {
            feedback->feedback = "PullMoveAction failed during approach.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        // open gripper
        std_msgs::msg::String cmd;
        cmd.data = "o";
        gripper_pub_->publish(cmd);

        if (!linear_drop_action_.execute(move_group, approach_action_.getApproachGoal(goal, 0.005), goal_handle))
        {
            feedback->feedback = "PullMoveAction failed during drop.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        geometry_msgs::msg::Pose pull_pose = goal.pose;
        tf2::Quaternion orientation;
        tf2::fromMsg(pull_pose.orientation, orientation);
        orientation.normalize();

        tf2::Vector3 forward_vector(-1.0, 0.0, 0.0);
        tf2::Vector3 pull_vector = tf2::quatRotate(orientation, forward_vector).normalized() * pull_distance_;

        pull_pose.position.x += pull_vector.x();
        pull_pose.position.y += pull_vector.y();
        pull_pose.position.z += pull_vector.z();

        manipulation::action::Manipulation::Goal pull_goal = goal;
        pull_goal.pose = pull_pose;

        rclcpp::sleep_for(std::chrono::seconds(2));

        // close gripper
        cmd.data = "cp";
        gripper_pub_->publish(cmd);

        if (!linear_drop_action_.execute(move_group, approach_action_.getApproachGoal(goal, 0.01), goal_handle))
        {
            feedback->feedback = "PullMoveAction failed during drop.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        // pull
        feedback->feedback = "Executing pull with linear move...";
        goal_handle->publish_feedback(feedback);
        if (!linear_pull_action_.execute(move_group, pull_goal, goal_handle))
        {
            feedback->feedback = "PushMoveAction failed during pull.";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        rclcpp::sleep_for(std::chrono::seconds(1));

        feedback->feedback = "PullMoveAction completed successfully.";
        goal_handle->publish_feedback(feedback);
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> gripper_pub_;
    ApproachMoveAction approach_action_;
    LinearMoveAction linear_drop_action_;
    LinearMoveAction linear_pull_action_;
    double drop_height_;
    double pull_distance_;
};
