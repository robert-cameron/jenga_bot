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
            'baud': baud
        }]
    )

    return LaunchDescription([
        end_eff_node
    ])
