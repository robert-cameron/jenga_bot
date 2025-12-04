gnome-terminal --tab -t "Calibratio Point" -e './setup_calibration_point.sh'

sleep 5

source install/setup.bash

ros2 run object_detect camera_calibration
