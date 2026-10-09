import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wro2026_sim")
    turtlebot3_share = get_package_share_directory("turtlebot3_gazebo")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    world_file = os.path.join(
        package_share, "worlds", "wro2026_nav2_field.sdf"
    )
    phase7_config = os.path.join(package_share, "config", "phase7")
    bridge_file = os.path.join(phase7_config, "mapping_bridge.yaml")
    robot_description_file = os.path.join(
        phase7_config, "turtlebot3_burger_cam_tf.urdf"
    )
    slam_params_file = os.path.join(
        phase7_config, "slam_toolbox_mapping.yaml"
    )
    motion_supervisor = os.path.join(
        phase7_config, "mapping_motion_supervisor.py"
    )

    with open(robot_description_file, encoding="utf-8") as stream:
        robot_description = stream.read()

    resource_paths = [
        os.path.join(package_share, "models"),
        os.path.join(turtlebot3_share, "models"),
    ]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    gui = LaunchConfiguration("gui")
    motion_delay = LaunchConfiguration("motion_delay")

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
                executable="bridge_node",
                name="phase7_mapping_bridge",
                parameters=[{"config_file": bridge_file}],
                output="screen",
            )
        ],
    )
    robot_state_publisher = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="phase7_mapping_robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            )
        ],
    )
    slam = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        slam_toolbox_share, "launch", "online_async_launch.py"
                    )
                ),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                }.items(),
            )
        ],
    )
    line_detector = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="wro2026_sim",
                executable="line_detector.py",
                name="phase7_mapping_line_detector",
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )
    final_accepted_motion = TimerAction(
        period=motion_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    motion_supervisor,
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start Gazebo GUI when true.",
            ),
            DeclareLaunchArgument(
                "motion_delay",
                default_value="25.0",
                description=(
                    "Wall seconds before starting the byte-verified Final "
                    "Accepted controller motion harness."
                ),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)
            ),
            SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
            gazebo_server,
            gazebo_gui,
            bridge,
            robot_state_publisher,
            slam,
            line_detector,
            final_accepted_motion,
        ]
    )
