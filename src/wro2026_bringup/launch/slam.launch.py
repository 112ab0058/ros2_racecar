"""
Online mapping with slam_toolbox, READ ONLY (the motors cannot turn).

Simulation (with sim.launch.py running, ROS_DOMAIN_ID 42 / 43):
    ros2 launch wro2026_bringup slam.launch.py use_sim_time:=true
Real car (with localization.launch.py running; push the car by hand, Motor OFF):
    ros2 launch wro2026_bringup slam.launch.py
Save the map (another terminal):
    ros2 run nav2_map_server map_saver_cli -f ~/maps/field --ros-args -p save_map_timeout:=10.0
  (wall clock is fine; the default 2 s timeout sometimes missed the map)

  slam_toolbox (async): /scan + TF odom -> base_link (from the EKF) -> /map, TF map -> odom
  lifecycle_manager_slam: configures + activates slam_toolbox. slam_toolbox's own
      event-based autostart sometimes left it in "configuring" (no map; sim 2026-10-21).
Settings and why: config/slam_toolbox.yaml. Stop with Ctrl+C.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wro2026_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('params'),
            'use_lifecycle_manager': 'true',
        }.items())
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['slam_toolbox'], 'bond_timeout': 4.0}])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params',
                              default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')),
        slam,
        manager,
    ])
