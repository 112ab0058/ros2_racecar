"""
N10P scan node (READ ONLY).

ros2 launch wro2026_lidar_n10p scan.launch.py port:=/dev/...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wro2026_lidar_n10p'), 'config',
                          'n10p.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', description='N10P serial device (required)'),
        Node(
            package='wro2026_lidar_n10p',
            executable='scan_node',
            name='n10p_scan',
            output='screen',
            parameters=[config, {'port': LaunchConfiguration('port')}],
        ),
    ])
