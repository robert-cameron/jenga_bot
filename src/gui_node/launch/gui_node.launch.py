from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('gui_node'),
        'launch',
        'jenga_config.rviz'
    )

    return LaunchDescription([
        Node(
            package='gui_node',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
        Node(
            package='os_node',           # 如果 os_node 是单独的包且需要
            executable='os_node',        # 改成实际的可执行文件名
            name='os_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        )
    ])
