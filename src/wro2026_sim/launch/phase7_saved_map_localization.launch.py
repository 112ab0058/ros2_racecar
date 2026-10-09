import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wro2026_sim")
    turtlebot3_share = get_package_share_directory("turtlebot3_gazebo")
    world_file = os.path.join(
        package_share, "worlds", "wro2026_nav2_field.sdf"
    )
    phase7_config = os.path.join(package_share, "config", "phase7")
    bridge_file = os.path.join(phase7_config, "mapping_bridge.yaml")
    robot_description_file = os.path.join(
        phase7_config, "turtlebot3_burger_cam_tf.urdf"
    )
    map_server_file = os.path.join(phase7_config, "map_server.yaml")
    amcl_file = os.path.join(phase7_config, "amcl.yaml")

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
    localization_delay = LaunchConfiguration("localization_delay")

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
                name="phase7_localization_bridge",
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
                name="phase7_localization_robot_state_publisher",
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
    localization = TimerAction(
        period=localization_delay,
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[map_server_file],
                output="screen",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[amcl_file],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                        "bond_timeout": 4.0,
                    }
                ],
                output="screen",
            ),
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
                "localization_delay",
                default_value="6.0",
                description=(
                    "Wall seconds before starting only Map Server, AMCL, "
                    "and their lifecycle manager."
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
            localization,
        ]
    )
