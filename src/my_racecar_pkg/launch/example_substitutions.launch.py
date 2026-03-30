from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

def generate_launch_description():
    # 定義接收變數
    car_ns = LaunchConfiguration('car_ns')
    
    # 宣告可從外部傳入的參數
    car_ns_launch_arg = DeclareLaunchArgument(
        'car_ns', default_value='default_car'
    )

    # 使用參數執行 shell 指令 (例如啟動節點或設定參數)
    msg_cmd = ExecuteProcess(
        cmd=[['echo "啟動賽車命名空間: ", ', car_ns]],
        shell=True
    )

    return LaunchDescription([
        car_ns_launch_arg,
        msg_cmd
    ])