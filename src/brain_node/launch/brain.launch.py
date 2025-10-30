from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brain_node',
            executable='brain',
            name='brain',
            output='screen',
            parameters=[
                {'open_gap_mm': 30.0},
                {'close_gap_mm': 5.0},
                {'approach_action': 'linear_move'},
                {'lift_action': 'free_move'},
                {'lift_dz': 0.05},
            ],
        ),
    ])
