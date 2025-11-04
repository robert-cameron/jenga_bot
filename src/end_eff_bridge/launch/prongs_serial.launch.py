from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='end_eff_bridge',
            executable='bridge',
            name='end_eff_serial_bridge',
            output='screen',
            parameters=[
                {'port': ''},       # auto-pick USB if empty
                {'baud': 115200},
                {'auto_hint': 'usb'}
            ]
        )
    ])
