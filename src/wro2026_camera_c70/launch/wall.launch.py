"""
C70 wall node (READ ONLY: camera only).

ros2 launch wro2026_camera_c70 wall.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wro2026_camera_c70'), 'config',
                          'c70_wall.yaml')
    return LaunchDescription([
        Node(
            package='wro2026_camera_c70',
            executable='wall_node',
            name='c70_wall',
            output='screen',
            parameters=[config],
        ),
    ])
