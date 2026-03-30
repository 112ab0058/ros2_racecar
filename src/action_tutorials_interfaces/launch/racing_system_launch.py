from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 啟動模擬感測器：將數據發布到 /sensor_data
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='mock_sensor',
            remappings=[('/chatter', '/sensor_data')] # 重新映射話題
        ),
        # 啟動路徑規劃器：監聽 /sensor_data 並運算
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='path_planner',
            remappings=[('/chatter', '/sensor_data')]
        )
    ])