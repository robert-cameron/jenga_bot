import launch
import os
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description_custom"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description_custom"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description_custom"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description_custom"), "config", "ur5e", "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description_custom"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2 ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=true ",
            "safety_pos_margin:=0.15 ",
            "safety_k_position:=20 ",
            "name:=ur ",
            "ur_type:=ur5e ",
            "prefix:=",
            '""',
        ]
    )

    return {"robot_description": robot_description_content}


def get_robot_description_semantic():
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config_custom"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=ur ",
            "prefix:=",
            '""',
        ]
    )
    return {"robot_description_semantic": robot_description_semantic_content}


def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    manipulation_node = Node(
        package="manipulation",
        executable="manipulation_node",
        name="manipulation_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([manipulation_node])
