source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

gnome-terminal --tab -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=yyy.yyy.yyy.yyy \
    initial_joint_controller:=joint_trajectory_controller \
    use_fake_hardware:=true \
    launch_rviz:=false \
    description_package:=ur_description_custom \
    description_file:=ur.urdf.xacro'
sleep 5

gnome-terminal --tab -t "MoveitServer" -e 'ros2 launch ur_moveit_config_custom ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true \
    use_fake_hardware:=true \
    description_package:=ur_description_custom \
    description_file:=ur.urdf.xacro'
    
sleep 5

gnome-terminal --tab -t "ManipulationServer" -e 'ros2 launch manipulation manipulation_launch.py'

gnome-terminal --tab -t "EndEffServer" -e 'ros2 run end_eff_bridge bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200'

sleep 5

gnome-terminal --tab -t "BrainServer" -e 'ros2 launch brain_node brain.launch.py'

# gnome-terminal --tab -t "EndEffServer" -e 'ros2 run end_eff_bridge bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200'

# useful terminal commands 

# pushing blocks
# ros2 action send_goal /manipulation_action manipulation/action/Manipulation  "{action_type: 'push_move', pose: {position: {x: 0.45, y: 0.18, z: 0.275}, orientation: {x: 0.707107, y: 0.707107, z: 0.0, w: 0.0}}}"
# ros2 action send_goal /manipulation_action manipulation/action/Manipulation  "{action_type: 'push_move', tf: 'block3'}"

# make a tf
# ros2 run tf2_ros static_transform_publisher 0.6 0.1 0.15 0 0 0 base_link block
