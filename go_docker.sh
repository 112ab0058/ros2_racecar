#!/bin/bash
export ROS_DOMAIN_ID=50
pkill -f udp_bridge_jetson 2>/dev/null
pkill -f turtle_tf2_broadcaster 2>/dev/null
pkill -f carrot_broadcaster 2>/dev/null
pkill -f carrot_tf_listener 2>/dev/null
pkill -f time_travel_listener 2>/dev/null
pkill -f lidar_tf 2>/dev/null
pkill -f lidar_sim 2>/dev/null
pkill -f lidar_obstacle_detector 2>/dev/null
sleep 1

ROS_DOMAIN_ID=50 python3 /workspace/src/my_racecar_pkg/my_racecar_pkg/udp_bridge_jetson.py > /tmp/bridge.log 2>&1 &
ROS_DOMAIN_ID=50 python3 /workspace/src/my_racecar_pkg/my_racecar_pkg/turtle_tf2_broadcaster.py --ros-args -p turtlename:=turtle1 > /dev/null 2>&1 &
ROS_DOMAIN_ID=50 python3 /workspace/src/my_racecar_pkg/my_racecar_pkg/turtle_tf2_broadcaster.py --ros-args -p turtlename:=turtle2 > /dev/null 2>&1 &
ROS_DOMAIN_ID=50 python3 /workspace/src/carrot_broadcaster.py > /dev/null 2>&1 &
sleep 2
ROS_DOMAIN_ID=50 python3 /workspace/src/carrot_tf_listener.py > /tmp/listener.log 2>&1 &

# 雷達相關
ROS_DOMAIN_ID=50 python3 /workspace/src/my_racecar_pkg/my_racecar_pkg/lidar_tf.py > /tmp/lidar_tf.log 2>&1 &
sleep 1
ROS_DOMAIN_ID=50 python3 /workspace/src/lidar_sim.py > /tmp/lidar_sim.log 2>&1 &
ROS_DOMAIN_ID=50 python3 /workspace/src/lidar_obstacle_detector.py > /tmp/lidar_detector.log 2>&1 &

echo "✅ 完成"
