gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=false launch_rviz:=false'

sleep 10

gnome-terminal -t "MoveitServer" -e 'ros2 launch ur_moveit_config_custom ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true'

sleep 10

ros2 launch manipulation manipulation_launch.py

# ros2 action send_goal /manipulation_action manipulation/action/Manipulation  "{action_type: 'push_move', block_pose: {position: {x: 0.45, y: 0.18, z: 0.275}, orientation: {x: 0.707107, y: 0.707107, z: 0.0, w: 0.0}}}"

