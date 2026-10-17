"""
Publish the R550 TF tree (base_link -> laser_link, imu_link, base_footprint).

ros2 launch wro2026_description description.launch.py

robot_state_publisher reads the URDF and publishes the fixed transforms on
/tf_static (and the URDF itself on /robot_description). It does not touch
any hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('wro2026_description'), 'urdf',
                        'r550.urdf')
    with open(urdf) as f:
        robot_description = f.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
