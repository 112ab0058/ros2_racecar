import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value="/opt/ros/jazzy/share/turtlebot3_gazebo/models",
    )

    world_file = os.path.expanduser(
        "~/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf"
    )

    urdf_file = "/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf"
    with open(urdf_file, "r") as f:
        robot_desc = f.read().replace("${namespace}", "")

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    # 2. Bridge（延遲 5 秒）
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                    "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
                    "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                ],
                output="screen",
            )
        ],
    )

    ground_truth_odom = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="wro2026_sim",
                executable="gazebo_ground_truth_odom.py",
                parameters=[
                    {"use_sim_time": True},
                    {"gz_pose_topic": "/world/wro2026/dynamic_pose/info"},
                    {"model_name": "turtlebot3_burger"},
                    {"odom_topic": "/odom"},
                    {"odom_frame": "odom"},
                    {"child_frame": "base_footprint"},
                    {"publish_tf": True},
                ],
                output="screen",
            )
        ],
    )

    # 3. robot_state_publisher（延遲 8 秒）
    rsp = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_desc},
                    {"use_sim_time": True},
                ],
                output="screen",
            )
        ],
    )

    # 4. SLAM Toolbox（延遲 15 秒）
    slam = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "launch", "slam_toolbox",
                    "online_async_launch.py",
                    "slam_params_file:=/home/ray/ros2_ws/src/wro2026_sim/config/slam_params.yaml",
                    "use_sim_time:=true",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        set_gz_resource,
        gazebo,
        bridge,
        ground_truth_odom,
        rsp,
        slam,
    ])
