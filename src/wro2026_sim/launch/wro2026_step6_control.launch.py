import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # LiDAR/odom debug baseline only. This launch does not start the camera
    # bridge or line_detector, so it is not the official line-triggered
    # Step 6/7 full-lap test launch.
    package_dir = os.path.expanduser("~/ros2_ws/src/wro2026_sim")
    world_file = os.path.join(package_dir, "worlds", "wro2026_field.sdf")
    line_follower_script = os.path.join(package_dir, "scripts", "line_follower.py")

    resource_path = (
        "/home/ray/ros2_ws/src/wro2026_sim/models:"
        "/opt/ros/jazzy/share/turtlebot3_gazebo/models"
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
                output="screen",
            )
        ],
    )

    ground_truth_odom = TimerAction(
        period=3.0,
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

    line_follower = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", line_follower_script],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            gazebo,
            bridge,
            ground_truth_odom,
            line_follower,
        ]
    )
