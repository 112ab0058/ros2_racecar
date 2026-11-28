"""
Nav2 on a saved map (AMCL + map_server + planner / controller / behaviors). PUBLISHES /cmd_vel.

Simulation (with sim.launch.py running, ROS_DOMAIN_ID 42 / 43):
    ros2 launch wro2026_bringup nav2.launch.py use_sim_time:=true \
        map:=$(ros2 pkg prefix wro2026_sim_r550)/share/wro2026_sim_r550/maps/field_sim.yaml
    ros2 run wro2026_sim_r550 nav_lap_node --ros-args -p use_sim_time:=true   # goals + checks
Real car: NOT YET. /cmd_vel only moves the car through base_node with enable_commands:=true;
    the first floor test needs its own plan and Ray's go-ahead (CLAUDE.md safety rules).

  nav2_bringup/bringup_launch.py with config/nav2_r550.yaml (SmacHybrid Dubins + RPP, no
  spin, Twist), the BT files config/bt_r550_*.xml, map frame = the map file's frame
  (field_sim.yaml: field centre 0, 0). AMCL starts at the sim spawn pose (nav2_r550.yaml).
Stop with Ctrl+C (CommandGate's 0.3 s timeout then stops the car).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('wro2026_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'False',      # one process per server: clearer logs
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('map', description='map yaml (e.g. sim maps/field_sim.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params',
                              default_value=os.path.join(share, 'config', 'nav2_r550.yaml')),
        nav2,
    ])
