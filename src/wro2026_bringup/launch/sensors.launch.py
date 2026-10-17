"""
Start everything READ-ONLY on the car in one terminal.

ros2 launch wro2026_bringup sensors.launch.py
ros2 launch wro2026_bringup sensors.launch.py foxglove:=false

  r550_feedback          STM32 -> /odom, TF odom->base_link, /imu/data, /battery
  n10p_scan              N10P  -> /scan
  robot_state_publisher  URDF  -> TF base_link->laser_link, imu_link, base_footprint
  foxglove_bridge        ws://<orin>:8765, read only (clients cannot publish)

Nothing here writes to the STM32: the motors cannot turn. Stop with Ctrl+C.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Fixed names: both are CH9102 chips, only the serial number differs (2026-10-17).
STM32_PORT = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'
LIDAR_PORT = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00'


def _include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stm32_port', default_value=STM32_PORT),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('foxglove', default_value='true'),
        _include('wro2026_base_r550', 'feedback.launch.py',
                 {'port': LaunchConfiguration('stm32_port')}),
        _include('wro2026_lidar_n10p', 'scan.launch.py',
                 {'port': LaunchConfiguration('lidar_port')}),
        _include('wro2026_description', 'description.launch.py'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
            parameters=[{
                'port': 8765,
                # read only: no clientPublish / services / parameters
                'capabilities': ['connectionGraph', 'assets'],
                'client_topic_whitelist': ['^$'],
            }],
        ),
    ])
