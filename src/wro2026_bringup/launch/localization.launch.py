"""
Sensors + EKF, READ ONLY (the motors cannot turn).

ros2 launch wro2026_bringup localization.launch.py
ros2 launch wro2026_bringup localization.launch.py foxglove:=false

  everything in sensors.launch.py, but r550_feedback with publish_odom_tf:=false
  ekf_filter_node (robot_localization): /odom vx, vy + /imu/data vyaw
      -> /odometry/filtered and TF odom -> base_link (only the EKF publishes it)

Settings and why: config/ekf.yaml. Stop with Ctrl+C.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wro2026_bringup')
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sensors.launch.py')),
        launch_arguments={
            'publish_odom_tf': 'false',          # the EKF publishes odom -> base_link
            'foxglove': LaunchConfiguration('foxglove'),
        }.items())
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(share, 'config', 'ekf.yaml')],
    )
    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='true'),
        sensors,
        ekf,
    ])
