"""
Keyboard-driven mapping session on the real car: base, lidar, EKF, slam_toolbox, rosbag.

CAN MAKE THE MOTORS TURN with enable_commands:=true (default false = read only).
Normally started by tools/real/orin_map.sh, which asks first and runs the keyboard.

  r550_base              STM32 <-> /odom, /imu/data; /cmd_vel -> limits -> STM32, /cmd_vel_out
  n10p_scan              N10P -> /scan
  robot_state_publisher  URDF -> TF base_link -> laser_link
  ekf_filter_node        /odom + /imu/data -> TF odom -> base_link (the only publisher)
  slam_toolbox           /scan + TF -> /map, TF map -> odom
  foxglove_bridge        ws://<orin>:8765, read only (clients cannot publish)
  ros2 bag record        with bag_dir:=<new dir>: training data for the imitation network

Nothing here publishes /cmd_vel: only the keyboard (teleop_node) does.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

STM32_PORT = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'
LIDAR_PORT = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00'
BAG_TOPICS = ['/scan', '/cmd_vel', '/cmd_vel_out', '/odom', '/imu/data', '/odometry/filtered',
              '/tf', '/tf_static', '/battery']


def _include(package, launch_file, arguments=None, condition=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items(),
                                    condition=condition)


def generate_launch_description():
    share = get_package_share_directory('wro2026_bringup')
    bag_dir = LaunchConfiguration('bag_dir')
    return LaunchDescription([
        DeclareLaunchArgument('stm32_port', default_value=STM32_PORT),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('enable_commands', default_value='false'),
        DeclareLaunchArgument('max_vx', default_value='0.1'),
        DeclareLaunchArgument('max_run_time', default_value='300.0'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value=''),
        _include('wro2026_base_r550', 'base.launch.py',
                 {'port': LaunchConfiguration('stm32_port'),
                  'enable_commands': LaunchConfiguration('enable_commands'),
                  'max_vx': LaunchConfiguration('max_vx'),
                  'max_run_time': LaunchConfiguration('max_run_time'),
                  'publish_odom_tf': 'false'}),          # the EKF publishes odom -> base_link
        _include('wro2026_lidar_n10p', 'scan.launch.py',
                 {'port': LaunchConfiguration('lidar_port')}),
        _include('wro2026_description', 'description.launch.py'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'ekf.yaml')],
        ),
        _include('wro2026_bringup', 'slam.launch.py',
                 condition=IfCondition(LaunchConfiguration('slam'))),
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
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir, '--topics'] + BAG_TOPICS,
            output='screen',
            sigterm_timeout='8',
            condition=IfCondition(PythonExpression(["'", bag_dir, "' != ''"])),
        ),
    ])
