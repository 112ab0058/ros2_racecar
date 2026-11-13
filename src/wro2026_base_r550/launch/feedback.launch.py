"""
R550 feedback node (READ ONLY).

ros2 launch wro2026_base_r550 feedback.launch.py port:=/dev/...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wro2026_base_r550'), 'config',
                          'r550_feedback.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', description='C30D serial 3 device (required)'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        Node(
            package='wro2026_base_r550',
            executable='feedback_node',
            name='r550_feedback',
            output='screen',
            parameters=[config, {
                'port': LaunchConfiguration('port'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }],
        ),
    ])
