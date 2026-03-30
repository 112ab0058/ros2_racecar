#!/bin/bash
# 1. 強力清理所有相關 Python 行程
pkill -9 -f python3
pkill -9 -f udp_bridge
pkill -9 -f broadcaster
pkill -9 -f listener
# 延長等待時間至 3 秒，確保埠號完全釋放
sleep 3

export ROS_DOMAIN_ID=99
export PYTHONPATH=$PYTHONPATH:/workspace/src/my_racecar_pkg
PKG_PATH="/workspace/src/my_racecar_pkg/my_racecar_pkg"

echo "🚀 重啟 Jetson 追蹤系統 (Domain 99)..."

# 2. 啟動橋接器 (背景執行)
python3 $PKG_PATH/udp_bridge_jetson.py &
sleep 2

# 3. 啟動廣播器 (背景執行)
python3 $PKG_PATH/turtle_tf2_broadcaster.py --ros-args -p turtlename:=turtle1 &
sleep 1
python3 $PKG_PATH/turtle_tf2_broadcaster.py --ros-args -p turtlename:=turtle2 &
sleep 1

# 4. 啟動大腦 (保持在前台)
echo "🧠 啟動追逐邏輯..."
python3 $PKG_PATH/turtle_tf2_listener.py