import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 獲取參數檔路徑
    config = os.path.join(get_package_share_directory('my_racecar_pkg'), 'config', 'racecar_params.yaml')

    return LaunchDescription([
        # 1. 靜態座標廣播 (官方 C++ 版：最穩定)
        # 建立 base_link -> laser_frame 的連接，這是大腦導航的關鍵路徑
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fix_my_laser',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # 2. 模擬雷達 (發布 /scan 資料)
        Node(
            package='my_racecar_pkg', 
            executable='lidar_sim', 
            name='lidar_sim'
        ),

        # 3. 決策大腦 (載入 YAML 參數)
        Node(
            package='my_racecar_pkg', 
            executable='talker', 
            parameters=[config]
        ),

        # 4. 馬達驅動 (監聽 /cmd_vel)
        Node(
            package='my_racecar_pkg', 
            executable='listener', 
            name='motor_driver'
        ),
        
        # 5. 里程計廣播 (建立 odom -> base_link 的轉換)
        Node(
            package='my_racecar_pkg', 
            executable='odom_pub', 
            name='odom_pub'
        )

        # 註解掉原本的 Node 6，避免 laser_frame 出現多個父節點導致 TF 樹斷裂
    ])