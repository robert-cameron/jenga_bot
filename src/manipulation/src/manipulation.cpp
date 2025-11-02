#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "manipulation/action/manipulation.hpp"
#include "./actions/free_move_action.hpp"
#include "./actions/push_move_action.hpp"
#include "./actions/linear_move_action.hpp"
#include "./actions/approach_move_action.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using Manipulation = manipulation::action::Manipulation;
using GoalHandleManipulation = rclcpp_action::ServerGoalHandle<Manipulation>;

class ManipulationNode : public rclcpp::Node
{
public:
  ManipulationNode()
      : Node("manipulation_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
        move_group_interface_(std::shared_ptr<rclcpp::Node>(this, [](auto *) {}), "ur_manipulator"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    action_server_ = rclcpp_action::create_server<Manipulation>(
        this,
        "manipulation_action",
        std::bind(&ManipulationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ManipulationNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&ManipulationNode::handle_accepted, this, std::placeholders::_1));

    move_group_interface_.setEndEffectorLink("end_eff_contact");
    move_group_interface_.setPlanningTime(15.0);
    move_group_interface_.setNumPlanningAttempts(30);
    move_group_interface_.setPlannerId("LBKPIECEkConfigDefault");
    setupCollisionObjects();
  }

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Manipulation::Goal> goal)
  {
    std::string source_info = goal->tf.empty() ? "pose input" : "TF frame: " + goal->tf;
    RCLCPP_INFO(this->get_logger(), "Received goal (%s) for action '%s'",
                source_info.c_str(), goal->action_type.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleManipulation>)
  {
    RCLCPP_INFO(this->get_logger(), "Canceling current goal");
    move_group_interface_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleManipulation> goal_handle)
  {
    std::thread{std::bind(&ManipulationNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleManipulation> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    geometry_msgs::msg::Pose target_pose;

    if (!goal->tf.empty())
    {
      try
      {
        geometry_msgs::msg::TransformStamped tf_stamped =
            tf_buffer_.lookupTransform("world", goal->tf, tf2::TimePointZero);
        tf2::doTransform(geometry_msgs::msg::Pose(), target_pose, tf_stamped);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
        auto result = std::make_shared<Manipulation::Result>();
        result->result = false;
        goal_handle->abort(result);
        return;
      }
    }
    else
    {
      target_pose = goal->pose;
    }

    std::unique_ptr<BaseAction> action;
    if (goal->action_type == "push_move")
    {
      action = std::make_unique<PushMoveAction>(shared_from_this(), getEndEffectorPose());
    }
    else if (goal->action_type == "free_move")
    {
      action = std::make_unique<FreeMoveAction>();
    }
    else if (goal->action_type == "constrained_move")
    {
      action = std::make_unique<ConstrainedMoveAction>();
    }
    else if (goal->action_type == "linear_move")
    {
      action = std::make_unique<LinearMoveAction>();
    }
    else if (goal->action_type == "approach_move")
    {
      action = std::make_unique<ApproachMoveAction>(shared_from_this(), getEndEffectorPose());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown action type: %s", goal->action_type.c_str());
      return;
    }

    auto new_goal = *goal;
    new_goal.pose = target_pose;

    bool success = action->execute(move_group_interface_, new_goal, goal_handle);

    auto result = std::make_shared<Manipulation::Result>();
    result->result = success;
    if (success)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }

  geometry_msgs::msg::Pose getEndEffectorPose()
  {
    geometry_msgs::msg::Pose pose;
    try
    {
      // Lookup the transform from world -> end_eff_contact
      geometry_msgs::msg::TransformStamped tf_stamped =
          tf_buffer_.lookupTransform("world", "end_eff_contact", tf2::TimePointZero);

      // Convert Transform to Pose
      pose.position.x = tf_stamped.transform.translation.x;
      pose.position.y = tf_stamped.transform.translation.y;
      pose.position.z = tf_stamped.transform.translation.z;
      pose.orientation = tf_stamped.transform.rotation;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get end effector pose: %s", ex.what());
    }

    return pose;
  }

  moveit_msgs::msg::Constraints setOrientationDownConstraint()
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::OrientationConstraint ocm;

    ocm.link_name = move_group_interface_.getEndEffectorLink();
    ocm.header.frame_id = move_group_interface_.getPlanningFrame();
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

  moveit_msgs::msg::JointConstraint createJointConstraint(
      const std::string &joint_name, double position, double tolerance_above,
      double tolerance_below, double weight)
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = joint_name;
    jc.position = position;
    jc.tolerance_above = tolerance_above;
    jc.tolerance_below = tolerance_below;
    jc.weight = weight;
    return jc;
  }

  void setupCollisionObjects()
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string frame_id = move_group_interface_.getPlanningFrame();

    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.01, 0.85, 0.25, 0.013, frame_id, "table"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.2, frame_id, "ceiling"));
    // planning_scene_interface.applyCollisionObject(generateCollisionObject(0.10, 0.10, 0.4, 0.25, 0.25, 0.2, "world", "tower"));
  }

  auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, const std::string &frame_id, const std::string &id) -> moveit_msgs::msg::CollisionObject
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {sx, sy, sz};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }

  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<Manipulation>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
