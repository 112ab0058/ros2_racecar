"""Kept for compatibility; same arguments as bringup.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = os.path.join(
        get_package_share_directory('wro2026_base_waverover'), 'launch', 'bringup.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup)),
    ])
