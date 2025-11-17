from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baud = LaunchConfiguration('baud', default='115200')

    end_eff_node = Node(
        package='end_eff_bridge',
        executable='bridge',
        name='end_eff_serial_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
            'open_left_deg': 150.0,
            'open_right_deg': 30.0,
            'cp_left_deg': 55.0,
            'cp_right_deg': 115.0,
            'cf_left_deg': 0.0,
            'cf_right_deg': 180.0
        }]

    )

    return LaunchDescription([
        end_eff_node
    ])
