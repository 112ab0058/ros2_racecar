"""
R550 in Gazebo on the 160 cm field. SIMULATION ONLY (no hardware is touched).

    export ROS_DOMAIN_ID=42      # keep the simulation off the real car's ROS graph
    ros2 launch wro2026_sim_r550 sim.launch.py              # headless + EKF + Foxglove
    ros2 launch wro2026_sim_r550 sim.launch.py ekf:=false   # /odom publishes the TF itself

Starts:
  gz sim (server only, headless rendering for the GPU lidar) with worlds/r550_field.sdf
  the R550 model at (x, y, yaw), default south lane (-0.60, -0.55, 0)
  ros_gz_bridge (config/bridge.yaml), robot_state_publisher (the REAL car's URDF)
  sim_base_node    /cmd_vel -> the real CommandGate -> /sim/cmd_vel
  sim_sensors_node /sim/odom, /sim/imu -> /odom, /imu/data, /imu/data_raw (real covariances, ZUPT)
  ekf_filter_node  wro2026_bringup/config/ekf.yaml (same as the car)
  foxglove_bridge  read only, port 8765
All with use_sim_time. Refuses to start when a real chassis / lidar node is running.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REAL_NODES = 'wro2026_base_r550/(base_node|feedback_node)|wro2026_lidar_n10p/scan_node'


def _refuse_with_real_car(context):
    found = subprocess.run(['pgrep', '-af', REAL_NODES], capture_output=True, text=True).stdout
    if found.strip():
        raise RuntimeError('real car nodes are running, stop them before the simulation:\n'
                           + found)
    return []


def _sim(context):
    share = get_package_share_directory('wro2026_sim_r550')
    arg = context.launch_configurations
    world = os.path.join(share, 'worlds', 'r550_field.sdf')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -s --headless-rendering -v 2 ' + world,
                          'on_exit_shutdown': 'true'}.items())
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'r550_field', '-name', 'r550',
                            '-file', os.path.join(share, 'models', 'r550', 'model.sdf'),
                            '-x', arg['x'], '-y', arg['y'], '-z', '0.04', '-Y', arg['yaw']])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                               'use_sim_time': True}])
    with open(os.path.join(get_package_share_directory('wro2026_description'), 'urdf',
                           'r550.urdf')) as f:
        urdf = f.read()
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    ekf_on = arg['ekf'].lower() == 'true'
    base = Node(package='wro2026_sim_r550', executable='sim_base_node', output='screen',
                parameters=[{'use_sim_time': True, 'max_vx': float(arg['max_vx'])}])
    sensors = Node(package='wro2026_sim_r550', executable='sim_sensors_node', output='screen',
                   parameters=[{'use_sim_time': True, 'publish_odom_tf': not ekf_on}])
    actions = [gz, spawn, bridge, rsp, base, sensors]
    if ekf_on:
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('wro2026_bringup'),
                                     'config', 'ekf.yaml'), {'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-0.60'),
        DeclareLaunchArgument('y', default_value='-0.55'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('max_vx', default_value='0.3'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(
            get_package_share_directory('wro2026_sim_r550'), 'models')),
        OpaqueFunction(function=_refuse_with_real_car),
        OpaqueFunction(function=_sim),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             output='screen', condition=IfCondition(LaunchConfiguration('foxglove')),
             parameters=[{'port': 8765, 'use_sim_time': True,
                          'capabilities': ['connectionGraph', 'assets'],
                          'client_topic_whitelist': ['^$']}]),
    ])
