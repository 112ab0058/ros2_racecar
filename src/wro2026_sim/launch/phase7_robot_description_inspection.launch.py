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
    world_file = os.path.join(package_share, "worlds", "wro2026_nav2_field.sdf")
    phase7_config = os.path.join(package_share, "config", "phase7")
    bridge_file = os.path.join(phase7_config, "bridge.yaml")
    robot_description_file = os.path.join(
        phase7_config, "turtlebot3_burger_cam_tf.urdf"
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
                name="phase7_contract_bridge",
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
                name="phase7_robot_state_publisher",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description=(
                    "Inspect the Phase 7 TurtleBot3 TF/sensor/odom contract; "
                    "no controller or navigation/localization stack is launched."
                ),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)
            ),
            gazebo_server,
            gazebo_gui,
            bridge,
            robot_state_publisher,
        ]
    )
