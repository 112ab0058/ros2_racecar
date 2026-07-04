from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "physical_bringup is a placeholder for encoder/IMU/CAN/MCU "
                    "integration. Use race.launch.py for the current Gazebo workflow."
                )
            )
        ]
    )
