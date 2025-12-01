source /opt/ros/humble/setup.bash
source install/setup.bash

gnome-terminal --tab -t "Realsense" -e 'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable=true rgb_camera.color_profile:=1280x720x6'

gnome-terminal --tab -t "Static Camera" -e 'ros2 run tf2_ros static_transform_publisher 1.27677 0.0175114 0.673798 -0.414096 -0.019425 0.910018 0.003764 base_link camera_link'

gnome-terminal --tab -t "Real Tower" -e './realTower.sh'

gnome-terminal --tab -t "Calibration" -e './setup_calibration_point.sh'

sleep 5

gnome-terminal --tab -t "Object Detection" -e 'ros2 run object_detect object_detect.py'

echo "Vison setup, position end effector next to block42b and press enter to generate correct static transform"
read -p "Press Enter to continue..."

gnome-terminal --tab -t "Camera Calibration" -e 'ros2 run object_detect camera_calibration'

