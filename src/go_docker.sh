#!/bin/bash

source /opt/ros/foxy/install/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=50

echo "🐳 Docker 側節點啟動中..."

python3 /workspace/src/my_racecar_pkg/my_racecar_pkg/turtle_tf2_broadcaster.py --ros-args -p turtlename:=turtle1 > /dev/null 2>&1 &
python3 /workspace/src/my_racecar_pkg/my_racecar_pkg/turtle_tf2_broadcaster.py --ros-args -p turtlename:=turtle2 > /dev/null 2>&1 &
python3 /workspace/src/carrot_broadcaster.py > /dev/null 2>&1 &
sleep 2
python3 /workspace/src/carrot_tf_listener.py > /dev/null 2>&1 &

echo "✅ 完成！"
sleep 2
ros2 node list
