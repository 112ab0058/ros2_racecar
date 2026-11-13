"""
R550 in Gazebo on the 160 cm field. SIMULATION ONLY (no hardware is touched).

    export ROS_DOMAIN_ID=42      # keep the simulation off the real car's ROS graph
    ros2 launch wro2026_sim_r550 sim.launch.py              # headless + EKF + Foxglove
    ros2 launch wro2026_sim_r550 sim.launch.py ekf:=false   # /odom publishes the TF itself
    ros2 launch wro2026_sim_r550 sim.launch.py gui:=true foxglove:=false   # WSL: Gazebo window
    ros2 launch wro2026_sim_r550 sim.launch.py seed:=3 steer_p_gain:=5.0   # see below

Optional (empty = the gz default):
  seed          gz random seed (sensor noise, IMU bias sign): repeatable runs
  steer_p_gain  steering servo gain of the gz Ackermann plugin (1/s): the steering angle
                follows the command like a first-order lag with time constant 1 / gain.
                Default 20 (tau ~0.05 s) = the REAL R550 servo, measured 2026-10-28 with
                the wheels in the air (240 fps video: 0.10-0.13 s from start to end of a
                full swing). Empty = the gz default 1.0 (~1 s, far too slow: it caused the
                5-8 cm tracking errors of the 2026-10-27 campaign).
  ekf_params    another robot_localization yaml (sim experiments; default the car's ekf.yaml)
  obstacles     boxes on the field, "x,y;x,y" in the field frame (obstacle_size, default
                0.05 x 0.05 x 0.20 m: WRO sign footprint, tall enough for the lidar plane)
  gyro_scale    sim_sensors_node: gyro z x this (default 1.0; 1.005 = calibrated sim gyro)
  odom_vx_scale sim_sensors_node: /odom vx x this (default 1.0; 0.981 = calibrated sim odom)

Starts:
  gz sim (server only, headless rendering for the GPU lidar; gui:=true adds the Gazebo
  window, for a PC with a screen, e.g. tools/wsl/sim_r550_view.sh) with worlds/r550_field.sdf
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


def model_sdf(path, steer_p_gain=''):
    """Return the car model's SDF text, with the plugin's steer_p_gain set if given."""
    with open(path) as f:
        text = f.read()
    if steer_p_gain:
        tag = '<wheel_base>'
        if tag not in text:
            raise RuntimeError('no %s in %s' % (tag, path))
        text = text.replace(tag, '<steer_p_gain>%g</steer_p_gain>\n      %s'
                            % (float(steer_p_gain), tag), 1)
    return text


def obstacle_sdf(i, x, y, size):
    """Return the SDF of one static box obstacle standing on the floor at x, y."""
    sx, sy, sz = size
    geometry = '<geometry><box><size>%g %g %g</size></box></geometry>' % (sx, sy, sz)
    return ('<?xml version="1.0"?><sdf version="1.10"><model name="obstacle_%d">'
            '<static>true</static><pose>%g %g %g 0 0 0</pose><link name="link">'
            '<collision name="c">%s</collision><visual name="v">%s<material>'
            '<ambient>0.8 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>'
            '</visual></link></model></sdf>' % (i, x, y, sz / 2, geometry, geometry))


def parse_obstacles(text):
    """Return [(x, y), ...] from 'x,y;x,y' (empty text: [])."""
    out = []
    for item in text.replace(' ', '').split(';'):
        if item:
            x, y = (float(v) for v in item.split(','))
            out.append((x, y))
    return out


def _sim(context):
    share = get_package_share_directory('wro2026_sim_r550')
    arg = context.launch_configurations
    world = os.path.join(share, 'worlds', 'r550_field.sdf')
    gz_flags = '-r -v 2 ' if arg['gui'].lower() == 'true' else '-r -s --headless-rendering -v 2 '
    if arg['seed']:
        gz_flags += '--seed %d ' % int(arg['seed'])
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags + world,
                          'on_exit_shutdown': 'true'}.items())
    model = os.path.join(share, 'models', 'r550', 'model.sdf')
    source = ['-string', model_sdf(model, arg['steer_p_gain'])] if arg['steer_p_gain'] \
        else ['-file', model]
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'r550_field', '-name', 'r550', *source,
                            '-x', arg['x'], '-y', arg['y'], '-z', '0.04', '-Y', arg['yaw']])
    size = [float(v) for v in arg['obstacle_size'].split(',')]
    boxes = [Node(package='ros_gz_sim', executable='create', output='screen',
                  arguments=['-world', 'r550_field', '-name', 'obstacle_%d' % i,
                             '-string', obstacle_sdf(i, x, y, size)])
             for i, (x, y) in enumerate(parse_obstacles(arg['obstacles']))]
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
                   parameters=[{'use_sim_time': True, 'publish_odom_tf': not ekf_on,
                                'gyro_scale': float(arg['gyro_scale']),
                                'odom_vx_scale': float(arg['odom_vx_scale'])}])
    actions = [gz, spawn, *boxes, bridge, rsp, base, sensors]
    if ekf_on:
        ekf_params = arg['ekf_params'] or os.path.join(
            get_package_share_directory('wro2026_bringup'), 'config', 'ekf.yaml')
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[ekf_params, {'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-0.60'),
        DeclareLaunchArgument('y', default_value='-0.55'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('max_vx', default_value='0.3'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('seed', default_value=''),
        DeclareLaunchArgument('steer_p_gain', default_value='20.0'),
        DeclareLaunchArgument('ekf_params', default_value=''),
        DeclareLaunchArgument('obstacles', default_value=''),
        DeclareLaunchArgument('obstacle_size', default_value='0.05,0.05,0.20'),
        DeclareLaunchArgument('gyro_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_vx_scale', default_value='1.0'),
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
