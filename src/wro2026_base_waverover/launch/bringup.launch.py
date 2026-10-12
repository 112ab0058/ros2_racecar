"""Start the WAVE ROVER driver.

  ros2 launch wro2026_base_waverover bringup.launch.py                       # serial /dev/ttyTHS1
  ros2 launch wro2026_base_waverover bringup.launch.py transport:=http host:=192.168.0.16

max_speed is a PWM fraction (0.5 = 100%), not m/s. Default 0.05 (~10%).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# name, default, type, description
ARGS = [
    ('transport', 'serial', str, 'serial | http'),
    ('port', '/dev/ttyTHS1', str, 'serial device'),
    ('baud', '115200', int, 'serial baud rate'),
    ('host', '192.168.4.1', str, 'ESP32 address for http transport'),
    ('max_speed', '0.05', float, 'wheel command cap, PWM fraction (0.5 = 100%)'),
    ('track_width', '0.17', float, 'left/right wheel distance (m), used for cmd_vel -> L/R'),
    ('cmd_timeout', '0.3', float, 'send stop if no /cmd_vel for this long (s)'),
    ('send_rate', '10.0', float, 'serial: T:1 resend rate (Hz)'),
    ('http_cmd_period', '0.5', float, 'http: resend unchanged T:1 every (s)'),
    ('http_timeout', '0.3', float, 'http request timeout (s)'),
    ('feedback_period', '0.2', float, 'poll T:130 and T:126 every (s)'),
    ('poll_base', 'true', bool, 'poll T:130 (battery, applied PWM)'),
    ('poll_imu', 'true', bool, 'poll T:126 (IMU)'),
    ('serial_continuous_feedback', 'false', bool, 'serial: send T:131 cmd 1 at start'),
    ('imu_frame_id', 'imu_link', str, 'frame of /imu/data'),
    ('battery_frame_id', 'base_link', str, 'frame of /battery'),
    ('low_battery_voltage', '11.0', float, 'warn below this voltage'),
]


def generate_launch_description():
    decls = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, _, h in ARGS]
    params = {n: ParameterValue(LaunchConfiguration(n), value_type=t) for n, _, t, _ in ARGS}
    node = Node(
        package='wro2026_base_waverover',
        executable='driver',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )
    return LaunchDescription(decls + [node])
