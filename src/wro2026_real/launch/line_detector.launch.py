"""
Start the parameterised line detector (observes only, never sends /cmd_vel).

  ros2 launch wro2026_real line_detector.launch.py [image_topic:=/camera/image_raw]

Needs a camera driver publishing sensor_msgs/Image (rgb8 / bgr8 / rgba8 /
bgra8). The camera model and driver are still UNKNOWN / not installed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _detector(context):
    params = [LaunchConfiguration('config').perform(context)]
    topic = LaunchConfiguration('image_topic').perform(context)
    if topic:
        params.append({'image_topic': topic})
    return [Node(package='wro2026_real', executable='line_detector', name='line_detector',
                 output='screen', emulate_tty=True, parameters=params)]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('wro2026_real'), 'config', 'line_detector.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='line detector parameter YAML'),
        DeclareLaunchArgument('image_topic', default_value='',
                              description='image topic (empty = YAML value)'),
        OpaqueFunction(function=_detector),
    ])
