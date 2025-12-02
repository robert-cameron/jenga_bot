source /opt/ros/humble/setup.bash
source install/setup.bash

gnome-terminal --tab -t "Realsense" -e 'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true rgb_camera.color_profile:=1280x720x6'

gnome-terminal --tab -t "Static Camera" -e 'ros2 run tf2_ros static_transform_publisher 1.27354 0.0326318 0.681138   -0.397486 -0.00834818 0.91757 -0.000592307 base_link camera_link'

gnome-terminal --tab -t "Real Tower" -e './real_tower.sh'

gnome-terminal --tab -t "Calibration" -e './setup_calibration_point.sh'

sleep 5

gnome-terminal --tab -t "Object Detection" -e 'ros2 run object_detect object_detect'

echo "Vison setup, position end effector next to block42b and press enter to generate correct static transform"
read -p "Press Enter to continue..."

gnome-terminal --tab -t "Camera Calibration" -e 'ros2 run object_detect camera_calibration'

