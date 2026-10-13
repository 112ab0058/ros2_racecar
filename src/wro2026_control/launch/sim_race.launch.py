"""Gazebo race run with the pose-free controller.

    ros2 launch wro2026_control sim_race.launch.py                 # CCW, SDF obstacles
    ros2 launch wro2026_control sim_race.launch.py direction:=cw
    ros2 launch wro2026_control sim_race.launch.py obstacles:="-0.3,0.42;0.0,0.58;0.3,0.42"
    ros2 launch wro2026_control sim_race.launch.py gui:=true

Starts Gazebo (server only unless gui:=true) with the wro2026_sim world, the
ROS-Gazebo bridge, the simulation odometry, the line detector, the race
controller and a monitor.  The monitor prints one RESULT line when the car
stops (laps complete or blocked) or the time limit passes, then the whole
launch shuts down.

Uses wro2026_sim unchanged.  /odom here is the simulation's ground-truth
odometry; the controller only takes yaw and travelled distance from it.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = "wro2026"
ROBOT = "turtlebot3_burger_cam"
OBSTACLE_MODELS = ["obstacle_yellow_mid", "obstacle_yellow_left", "obstacle_yellow_right"]
START = (-0.05, -0.50)


def _set_pose_cmd(name: str, x: float, y: float, z: float, yaw: float) -> ExecuteProcess:
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    return ExecuteProcess(
        cmd=["gz", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", req],
        output="screen")


def _setup(context):
    actions = []
    direction = LaunchConfiguration("direction").perform(context).strip().lower()
    if direction == "cw":
        actions.append(_set_pose_cmd(ROBOT, START[0], START[1], 0.01, math.pi))
    spec = LaunchConfiguration("obstacles").perform(context).strip()
    if spec:
        pairs = [p for p in spec.split(";") if p.strip()]
        for i, model in enumerate(OBSTACLE_MODELS):
            if i < len(pairs):
                x, y = (float(v) for v in pairs[i].split(","))
            else:
                x, y = 2.0 + i * 0.2, 2.0          # park unused obstacles outside the field
            actions.append(_set_pose_cmd(model, x, y, 0.075, 0.0))
    return actions


def generate_launch_description():
    sim_share = get_package_share_directory("wro2026_sim")
    ctl_share = get_package_share_directory("wro2026_control")
    tb3_share = get_package_share_directory("turtlebot3_gazebo")
    world_file = os.path.join(sim_share, "worlds", "wro2026_field.sdf")
    params_file = os.path.join(ctl_share, "config", "race_sim.yaml")

    resource_paths = [os.path.join(sim_share, "models"), os.path.join(tb3_share, "models")]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])

    gui = LaunchConfiguration("gui")
    sim_time = {"use_sim_time": True}

    gazebo_server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_file],
                                   condition=UnlessCondition(gui), output="screen")
    gazebo_gui = ExecuteProcess(cmd=["gz", "sim", "-r", world_file],
                                condition=IfCondition(gui), output="screen")

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen")

    odom = Node(
        package="wro2026_sim", executable="gazebo_ground_truth_odom.py",
        parameters=[sim_time,
                    {"gz_pose_topic": f"/world/{WORLD_NAME}/dynamic_pose/info"},
                    {"model_name": ROBOT}, {"odom_topic": "/odom"},
                    {"odom_frame": "odom"}, {"child_frame": "base_footprint"},
                    {"publish_tf": True}],
        output="screen")

    line_detector = Node(package="wro2026_sim", executable="line_detector.py",
                         parameters=[sim_time], output="screen")

    controller = Node(package="wro2026_control", executable="race_node",
                      parameters=[params_file, sim_time, {"start": True}],
                      output="screen")

    monitor = Node(package="wro2026_control", executable="race_monitor",
                   parameters=[sim_time,
                               {"time_limit": LaunchConfiguration("time_limit")},
                               {"start_x": START[0]}, {"start_y": START[1]}],
                   output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("direction", default_value="ccw",
                              description="ccw (spawn facing east) or cw (facing west)"),
        DeclareLaunchArgument("obstacles", default_value="",
                              description='"x,y;x,y;x,y" for up to 3 obstacles; empty keeps the SDF layout'),
        DeclareLaunchArgument("time_limit", default_value="300.0"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)),
        gazebo_server,
        gazebo_gui,
        TimerAction(period=2.0, actions=[bridge, odom]),
        TimerAction(period=3.0, actions=[OpaqueFunction(function=_setup)]),
        TimerAction(period=4.0, actions=[line_detector]),
        TimerAction(period=6.0, actions=[controller, monitor]),
        RegisterEventHandler(OnProcessExit(target_action=monitor,
                                           on_exit=[EmitEvent(event=Shutdown(reason="race finished"))])),
    ])
