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

using Manipulation = manipulation::action::Manipulation;
using GoalHandleManipulation = rclcpp_action::ServerGoalHandle<Manipulation>;

class ManipulationNode : public rclcpp::Node
{
public:
  ManipulationNode()
  : Node("manipulation_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group_interface_(std::shared_ptr<rclcpp::Node>(this, [](auto*){}), "ur_manipulator")
  {
    action_server_ = rclcpp_action::create_server<Manipulation>(
      this,
      "manipulation_action",
      std::bind(&ManipulationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManipulationNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ManipulationNode::handle_accepted, this, std::placeholders::_1)
    );

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
    RCLCPP_INFO(this->get_logger(),
                "Received goal: type='%s' pose=(%.2f, %.2f, %.2f)",
                goal->action_type.c_str(),
                goal->block_pose.position.x,
                goal->block_pose.position.y,
                goal->block_pose.position.z);
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
    auto feedback = std::make_shared<Manipulation::Feedback>();
    auto result   = std::make_shared<Manipulation::Result>();

    feedback->feedback = "Starting motion planning...";
    goal_handle->publish_feedback(feedback);

    if (goal->action_type != "free_move") {
      moveit_msgs::msg::Constraints constraints;

      //constraints.orientation_constraints.push_back(setOrientationDownConstraint().orientation_constraints[0]);

      constraints.joint_constraints.push_back(
        createJointConstraint("shoulder_pan_joint", 0.785398, 0.785398, 0.785398, 1.0));
      constraints.joint_constraints.push_back(
        createJointConstraint("shoulder_lift_joint", -1.0472, 0.523599, 0.523599, 1.0));
      constraints.joint_constraints.push_back(
        createJointConstraint("elbow_joint", 1.5708, 0.785398, 0.785398, 1.0));
      constraints.joint_constraints.push_back(
        createJointConstraint("wrist_1_joint", -1.5708, 0.785398, 0.785398, 1.0));
      constraints.joint_constraints.push_back(
        createJointConstraint("wrist_2_joint", -1.5708, 0.3, 0.3, 1.0));
      // constraints.joint_constraints.push_back(
      //   createJointConstraint("wrist_3_joint", 0, 1.5708, 1.5708, 1.0));

      move_group_interface_.setPathConstraints(constraints);
    } else {
      move_group_interface_.clearPathConstraints();
    }

    move_group_interface_.setStartStateToCurrentState();

    move_group_interface_.setPoseTarget(goal->block_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      feedback->feedback = "Plan found. Executing...";
      goal_handle->publish_feedback(feedback);
      move_group_interface_.execute(plan);
      feedback->feedback = "Motion complete.";
      goal_handle->publish_feedback(feedback);
      result->result = true;
      goal_handle->succeed(result);
    } else {
      feedback->feedback = "Planning failed.";
      goal_handle->publish_feedback(feedback);
      result->result = false;
      goal_handle->abort(result);
    }

    move_group_interface_.clearPoseTargets();
    move_group_interface_.clearPathConstraints();
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


    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 3.0, 0.70, -0.60, 0.5, frame_id, "backWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 2.4, 3.0, -0.55, 0.25, 0.8, frame_id, "sideWall"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(3, 3, 0.01, 0.85, 0.25, 0.05, frame_id, "table"));
    planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.5, frame_id, "ceiling"));
  }

    auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, const std::string& frame_id, const std::string& id) -> moveit_msgs::msg::CollisionObject {
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

  rclcpp_action::Server<Manipulation>::SharedPtr action_server_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
