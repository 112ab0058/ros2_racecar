"""
R550 base node: feedback + command side. CAN MAKE THE MOTORS TURN.

Read only (same as feedback.launch.py):
  ros2 launch wro2026_base_r550 base.launch.py
Wheels-in-the-air test (see docs/real/RUNBOOK.md, ask before running):
  ros2 launch wro2026_base_r550 base.launch.py enable_commands:=true max_vx:=0.1 max_run_time:=120

Do not run together with feedback.launch.py or sensors.launch.py's
feedback node: they open the same serial port.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import yaml

STM32_PORT = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'


def _node(context):
    share = get_package_share_directory('wro2026_base_r550')
    with open(os.path.join(share, 'config', 'r550_feedback.yaml')) as f:
        params = yaml.safe_load(f)['r550_feedback']['ros__parameters']
    with open(os.path.join(share, 'config', 'r550_base.yaml')) as f:
        params.update(yaml.safe_load(f)['r550_base']['ros__parameters'])

    def arg(name):
        return context.launch_configurations[name]

    params['port'] = arg('port')
    params['enable_commands'] = arg('enable_commands').lower() == 'true'
    params['max_vx'] = float(arg('max_vx'))
    params['max_run_time'] = float(arg('max_run_time'))
    params['min_battery_voltage'] = float(arg('min_battery_voltage'))
    params['publish_odom_tf'] = arg('publish_odom_tf').lower() == 'true'
    return [Node(package='wro2026_base_r550', executable='base_node', name='r550_base',
                 output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=STM32_PORT),
        DeclareLaunchArgument('enable_commands', default_value='false'),
        DeclareLaunchArgument('max_vx', default_value='0.3'),
        DeclareLaunchArgument('max_run_time', default_value='0.0'),
        DeclareLaunchArgument('min_battery_voltage', default_value='10.5'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        OpaqueFunction(function=_node),
    ])
