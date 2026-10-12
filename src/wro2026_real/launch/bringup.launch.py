"""
Real-car bringup: WAVE ROVER base driver + robot_state_publisher.

  ros2 launch wro2026_real bringup.launch.py                                  # UART /dev/ttyTHS1
  ros2 launch wro2026_real bringup.launch.py transport:=http host:=192.168.0.16

This launch only starts the driver and the robot description. Nothing in it
publishes /cmd_vel: the car moves only when a person sends commands
(e.g. teleop with the wheels off the ground). Per CLAUDE.md there is no
launch that drives the car autonomously on the ground.

Arguments transport / host / port / max_speed / cmd_timeout / config are
passed to wro2026_base_waverover's bringup.launch.py (empty = YAML value).
robot_state_publisher is skipped (with a message) if it is not installed.
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FORWARDED = [
    ('transport', 'serial | http (empty = YAML value, serial)'),
    ('host', 'ESP32 address for http (empty = YAML value)'),
    ('port', 'serial device (empty = YAML value)'),
    ('max_speed', 'wheel command cap, PWM fraction (empty = YAML value, 0.05)'),
    ('cmd_timeout', 'stop if no /cmd_vel for this long, s (empty = YAML value, 0.3)'),
    ('publish_odom_tf', 'TF odom -> base_link true/false (empty = YAML value, true)'),
]


def _has_package(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    base_share = get_package_share_directory('wro2026_base_waverover')
    real_share = get_package_share_directory('wro2026_real')
    urdf_file = os.path.join(real_share, 'urdf', 'wave_rover.urdf')
    with open(urdf_file, encoding='utf-8') as stream:
        robot_description = stream.read()

    decls = [DeclareLaunchArgument(
        'config', default_value=os.path.join(base_share, 'config', 'wave_rover.yaml'),
        description='base driver parameter YAML')]
    decls += [DeclareLaunchArgument(n, default_value='', description=h) for n, h in FORWARDED]
    decls.append(DeclareLaunchArgument(
        'use_robot_state_publisher', default_value='true',
        description='start robot_state_publisher with urdf/wave_rover.urdf'))

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_share, 'launch', 'bringup.launch.py')),
        launch_arguments=[(n, LaunchConfiguration(n)) for n, _ in [('config', '')] + FORWARDED],
    )

    actions = decls + [base]
    if _has_package('robot_state_publisher'):
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('use_robot_state_publisher')),
        ))
    else:
        actions.append(LogInfo(msg='robot_state_publisher is not installed: skipping it '
                                   '(only odom -> base_link TF will exist)'))
    return LaunchDescription(actions)
