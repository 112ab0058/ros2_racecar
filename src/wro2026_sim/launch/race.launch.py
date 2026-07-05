import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wro2026_sim")
    turtlebot3_share = get_package_share_directory("turtlebot3_gazebo")
    world_file = os.path.join(package_share, "worlds", "wro2026_field.sdf")

    resource_paths = [
        os.path.join(package_share, "models"),
        os.path.join(turtlebot3_share, "models"),
    ]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    gui = LaunchConfiguration("gui")

    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_file],
        condition=UnlessCondition(gui),
        output="screen",
    )

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        condition=IfCondition(gui),
        output="screen",
    )

    bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                output="screen",
            )
        ],
    )

    ground_truth_odom = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="wro2026_sim",
                executable="gazebo_ground_truth_odom.py",
                parameters=[
                    {"use_sim_time": True},
                    {"gz_pose_topic": "/world/wro2026/dynamic_pose/info"},
                    {"model_name": "turtlebot3_burger_cam"},
                    {"odom_topic": "/odom"},
                    {"odom_frame": "odom"},
                    {"child_frame": "base_footprint"},
                    {"publish_tf": True},
                ],
                output="screen",
            )
        ],
    )

    line_detector = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="wro2026_sim",
                executable="line_detector.py",
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    line_follower = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="wro2026_sim",
                executable="line_follower.py",
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start Gazebo with GUI when true; server-only when false.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)),
            gazebo_server,
            gazebo_gui,
            bridge,
            ground_truth_odom,
            line_detector,
            line_follower,
        ]
    )
