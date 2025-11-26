from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 找到配置文件路径
    rviz_config_file = os.path.join(
        get_package_share_directory('gui_node'),
        'launch',
        'jenga_config.rviz'
    )

    return LaunchDescription([
        # 启动你的 gui_node 节点
        Node(
            package='gui_node',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
        Node(
            package='os_node',           # 如果 os_node 是单独的包
            executable='os_node',        # 改成实际的可执行文件名
            name='os_node',
            output='screen'
        ),
        # 启动 RViz 并加载配置
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        )
    ])
