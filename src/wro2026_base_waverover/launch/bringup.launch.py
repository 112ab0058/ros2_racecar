"""
Start the WAVE ROVER base driver only.

  ros2 launch wro2026_base_waverover bringup.launch.py                       # serial /dev/ttyTHS1
  ros2 launch wro2026_base_waverover bringup.launch.py transport:=http host:=192.168.0.16

Parameters come from config/wave_rover.yaml (or config:=<file>). Every
driver parameter can be overridden with <name>:=<value>; an empty value
keeps the YAML value.

max_speed is a PWM fraction (0.5 = 100%), not m/s. Default 0.05 (~10%).
For the full robot (driver + robot_state_publisher) use
  ros2 launch wro2026_real bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name, type, description (defaults live in config/wave_rover.yaml)
OVERRIDES = [
    ('transport', str, 'serial | http'),
    ('port', str, 'serial device'),
    ('baud', int, 'serial baud rate'),
    ('host', str, 'ESP32 address for http transport'),
    ('max_speed', float, 'wheel command cap, PWM fraction (0.5 = 100%)'),
    ('track_width', float, 'left/right wheel distance (m), used for cmd_vel -> L/R'),
    ('cmd_timeout', float, 'send stop if no /cmd_vel for this long (s)'),
    ('send_rate', float, 'serial: T:1 resend rate (Hz)'),
    ('http_cmd_period', float, 'http: resend unchanged T:1 every (s)'),
    ('http_timeout', float, 'http request timeout (s)'),
    ('feedback_period', float, 'poll T:130 and T:126 every (s)'),
    ('poll_base', bool, 'poll T:130 (battery, applied PWM)'),
    ('poll_imu', bool, 'poll T:126 (IMU)'),
    ('serial_continuous_feedback', bool, 'serial: send T:131 cmd 1 at start'),
    ('imu_frame_id', str, 'frame of /imu/data'),
    ('battery_frame_id', str, 'frame of /battery'),
    ('low_battery_voltage', float, 'WARN below this voltage'),
    ('critical_battery_voltage', float, 'refuse non-zero commands below this voltage'),
    ('feedback_timeout', float, 'stop when no T:130 reply for this long (s); 0 disables'),
    ('diagnostics_period', float, '/diagnostics period (s)'),
    ('publish_odom', bool, 'publish /odom'),
    ('publish_odom_tf', bool, 'broadcast TF odom -> base_link'),
    ('odom_frame_id', str, 'odometry frame'),
    ('base_frame_id', str, 'robot base frame'),
    ('odom_rate', float, 'odometry rate (Hz)'),
    ('pwm_to_mps', float, 'm/s per unit wheel command (UNKNOWN, 0 = no translation)'),
    ('imu_timeout', float, 'max IMU age for odom yaw rate (s)'),
    ('imu_yaw_rate_sign', float, 'sign applied to IMU gz for odom'),
    ('odom_hold_when_stopped', bool, 'zero wheel command -> odom does not move'),
]


def _convert(value, kind):
    if kind is bool:
        if value.lower() in ('true', '1', 'yes'):
            return True
        if value.lower() in ('false', '0', 'no'):
            return False
        raise ValueError(f'not a bool: {value}')
    return kind(value)


def _driver(context):
    overrides = {}
    for name, kind, _ in OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = _convert(value, kind)
    params = [LaunchConfiguration('config').perform(context)]
    if overrides:
        params.append(overrides)
    return [Node(
        package='wro2026_base_waverover',
        executable='driver',
        name='wave_rover_driver',
        output='screen',
        emulate_tty=True,
        parameters=params,
    )]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('wro2026_base_waverover'), 'config', 'wave_rover.yaml')
    decls = [DeclareLaunchArgument('config', default_value=default_config,
                                   description='driver parameter YAML')]
    decls += [DeclareLaunchArgument(n, default_value='', description=f'{h} (empty = YAML value)')
              for n, _, h in OVERRIDES]
    return LaunchDescription(decls + [OpaqueFunction(function=_driver)])
