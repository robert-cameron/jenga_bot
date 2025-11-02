#pragma once

#include "base_action.hpp"
#include "constrained_move_action.hpp"
#include "linear_move_action.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ApproachMoveAction : public BaseAction
{
public:
  explicit ApproachMoveAction(rclcpp::Node::SharedPtr node,
                              geometry_msgs::msg::Pose end_eff_pose, double approach_distance = 0.02, double speed = 0.5)
      : node_(node),
        end_eff_pose(end_eff_pose),
        approach_distance(approach_distance),
        speed(speed)
  {
  }

  bool execute(
      moveit::planning_interface::MoveGroupInterface &move_group,
      const manipulation::action::Manipulation::Goal &goal,
      std::shared_ptr<GoalHandleManipulation> goal_handle) override
  {

    const manipulation::action::Manipulation::Goal approachGoal = getApproachGoal(goal);
    std::vector<manipulation::action::Manipulation::Goal> goals;

    // check if already at the desired pose
    if (posesEqual(approachGoal.pose, end_eff_pose))
    {
      auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
      feedback->feedback = "Skipping ApproachMoveAction - already at goal pose";
      goal_handle->publish_feedback(feedback);

      return true;
    }

    double diff = quaternionAngleDifference(approachGoal.pose.orientation, end_eff_pose.orientation);

    // check if goal is on the same face
    if (diff < 0.2) // approx 10deg
    {
      goals = {approachGoal};
    }

    // check if goal is on oppoisite face
    else if (diff > 2.9) // approx 170 deg
    {
    }

    // else, goal is on adjacent face
    else
    {
      geometry_msgs::msg::Point corner = findYAxisIntersection(end_eff_pose, approachGoal.pose);
      // broadcast a tf on the corner (used for debuging)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = node_->get_clock()->now();
      tf.header.frame_id = "world";
      tf.child_frame_id = "corner_frame";
      tf.transform.translation.x = corner.x;
      tf.transform.translation.y = corner.y;
      tf.transform.translation.z = corner.z;
      tf.transform.rotation = approachGoal.pose.orientation;

      tf_broadcaster_->sendTransform(tf);

      goals = {approachGoal, approachGoal};
      goals[0].pose.position = corner; // keep goal orientation
    }

    auto feedback = std::make_shared<manipulation::action::Manipulation::Feedback>();
    feedback->feedback = "Executing ApproachMoveAction...";
    goal_handle->publish_feedback(feedback);

    for (manipulation::action::Manipulation::Goal g : goals)
    {

      LinearMoveAction action(speed);
      if (!action.execute(move_group, g, goal_handle))
      {
        feedback->feedback = "ApproachMoveAction failed.";
        goal_handle->publish_feedback(feedback);
        return false;
      }
    }
    return true;
  }

  manipulation::action::Manipulation::Goal getApproachGoal(manipulation::action::Manipulation::Goal goal)
  {
    manipulation::action::Manipulation::Goal approachGoal = goal;

    // Convert geometry_msgs quaternion to tf2 quaternion
    tf2::Quaternion q;
    tf2::fromMsg(goal.pose.orientation, q);

    tf2::Vector3 backwards_vector(-1.0, 0.0, 0.0);
    tf2::Vector3 adjustment = tf2::quatRotate(q, backwards_vector).normalized() * approach_distance;

    approachGoal.pose.position.x += adjustment.x();
    approachGoal.pose.position.y += adjustment.y();
    approachGoal.pose.position.z += adjustment.z();

    return approachGoal;
  }

private:
  geometry_msgs::msg::Point findYAxisIntersection(
      const geometry_msgs::msg::Pose &pose1,
      const geometry_msgs::msg::Pose &pose2)
  {
    double yaw1 = getYawFromQuaternion(pose1.orientation);
    double yaw2 = getYawFromQuaternion(pose2.orientation);

    Eigen::Vector2d p1(pose1.position.x, pose1.position.y);
    Eigen::Vector2d d1(-sin(yaw1), cos(yaw1));
    Eigen::Vector2d p2(pose2.position.x, pose2.position.y);
    Eigen::Vector2d d2(-sin(yaw2), cos(yaw2));

    Eigen::Vector2d intersection;
    geometry_msgs::msg::Point result;
    if (findIntersection2D(p1, d1, p2, d2, intersection))
    {
      result.x = intersection.x();
      result.y = intersection.y();
      result.z = (pose1.position.z + pose2.position.z) / 2; // take the average height
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("Intersection"), "No intersection (parallel lines)");
      result.x = result.y = result.z = std::numeric_limits<double>::quiet_NaN();
    }
    return result;
  }

  bool findIntersection2D(
      const Eigen::Vector2d &p1, const Eigen::Vector2d &d1,
      const Eigen::Vector2d &p2, const Eigen::Vector2d &d2,
      Eigen::Vector2d &intersection)
  {
    Eigen::Matrix2d A;
    A << d1, -d2;
    if (fabs(A.determinant()) < 1e-6)
      return false; // Parallel lines (no intersection)

    Eigen::Vector2d b = p2 - p1;
    Eigen::Vector2d t = A.inverse() * b;
    intersection = p1 + t(0) * d1;
    return true;
  }

  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  bool posesEqual(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2, double pos_tolerance = 1e-3, double ang_tolerance = 1e-2)
  {
    // Compare position
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    double dz = p1.position.z - p2.position.z;
    double pos_diff = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Compare orientation using quaternion angular distance
    tf2::Quaternion q1, q2;
    tf2::fromMsg(p1.orientation, q1);
    tf2::fromMsg(p2.orientation, q2);

    double angle_diff = q1.angleShortestPath(q2);

    return (pos_diff < pos_tolerance) && (angle_diff < ang_tolerance);
  }

  double quaternionAngleDifference(
      const geometry_msgs::msg::Quaternion &q1_msg,
      const geometry_msgs::msg::Quaternion &q2_msg)
  {
    // Convert to tf2 quaternion
    tf2::Quaternion q1, q2;
    tf2::fromMsg(q1_msg, q1);
    tf2::fromMsg(q2_msg, q2);

    // Compute relative rotation: q_rel = q2 * q1.inverse()
    tf2::Quaternion q_rel = q2 * q1.inverse();

    // Get the angle of rotation (in radians)
    double angle = q_rel.getAngle();

    // Normalize to [0, π]
    if (angle > M_PI)
      angle = 2 * M_PI - angle;

    return angle;
  }

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::Pose end_eff_pose;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double approach_distance;
  double speed;
};
