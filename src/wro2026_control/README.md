# wro2026_control

不依賴場地座標的比賽 controller：車道置中、彎道、LiDAR 避障、自動判斷順/逆時針、數圈並在 3 圈後停回起點。模擬與實車共用同一份邏輯。

設計與舊 `line_follower.py` 的問題整理在 `docs/CONTROLLER_REVIEW_2026-10.md`。

## 輸入與輸出

| Topic | 型別 | 用途 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 牆、障礙物、車道中心、前方牆 |
| `/odom` | `nav_msgs/Odometry` | 只取 yaw 與行駛距離，不用絕對 x/y |
| `/imu`（選用） | `sensor_msgs/Imu` | `use_imu_yaw:=true` 時改用 IMU 的 yaw |
| `/line_detection` | `std_msgs/Float32MultiArray` | `line_detector.py` 的橘/藍觸發，只用來決定方向 |
| `/cmd_vel` | `geometry_msgs/Twist` | 輸出 |
| `/race_status` | `std_msgs/String`（JSON） | 狀態、彎數、圈數、停車原因 |

安全行為：參數 `start`（預設 false）設成 true 前不會動；scan 或 odom 超過 `sensor_timeout` 沒更新就送 0；模擬時間倒退會重置；結束（3 圈或 blocked）後持續送 0。

## 離線驗證（不需要 ROS / Gazebo）

需要 `numpy`、`opencv-python`、`shapely`、`matplotlib`（模擬器與繪圖用）。

```bash
cd src/wro2026_control
python3 -m pytest -q test                               # 單元測試 + 4 個閉迴路情境
python3 tools/sim2d.py --direction cw --scenario three --plot run.png
python3 tools/matrix.py                                 # 全部情境，平行執行
python3 tools/matrix.py --robust                        # yaw 漂移、里程比例誤差、雜訊、漏看線、Ackermann、不同起點
python3 tools/matrix.py --random 8                      # 隨機但合理的障礙物配置
python3 tools/matrix.py --stress                        # 過窄配置：只要求不碰撞
```

## Gazebo 驗證（WSL）

```bash
rsync -a --delete ~/ros2_racecar/src/wro2026_control/ ~/ros2_ws/src/wro2026_control/
cd ~/ros2_ws && colcon build --packages-select wro2026_control wro2026_sim && source install/setup.bash
ros2 launch wro2026_control sim_race.launch.py                     # 逆時針
ros2 launch wro2026_control sim_race.launch.py direction:=cw        # 順時針
ros2 launch wro2026_control sim_race.launch.py obstacles:="-0.30,0.40;0.15,0.60"
```

跑完會印一行 `RESULT {"result": "PASS", ...}`，launch 自動結束。

## 主要參數（`config/race_sim.yaml`，完整清單見 `core.py` 的 `Params`）

| 參數 | 預設 | 說明 |
|---|---|---|
| `cruise_speed` | 0.12 | 直線速度 (m/s) |
| `avoid_speed` | 0.07 | 路徑靠近障礙物時的速度 |
| `laps` | 3 | 圈數 |
| `lidar_x`、`body_front`、`body_rear`、`half_width` | TurtleBot3 | 車身尺寸，以 LiDAR 為基準 |
| `min_turn_radius` | 0.0 | 轉向輪車填最小轉彎半徑；0 表示差速車 |
| `hard_clearance` / `soft_clearance` | 0.018 / 0.07 | 規劃時車身與障礙物的最小 / 偏好間隙 (m) |
| `corner_radius` | 0.30 | 彎道參考弧半徑（上限為半個車道寬） |

## 實車注意

- 需要 LiDAR。WAVE ROVER 目前沒有光達，也沒有 encoder：行駛距離要由 IMU 加速度/指令積分或之後的 encoder 提供，yaw 用 `/imu`。
- 車身尺寸、`min_turn_radius` 要換成實車量測值。
- 依 `CLAUDE.md` 的安全規則：先在輪子架空狀態驗證 stop 與 timeout，再上地面。node 的 `start` 預設是 false（模擬 launch 才設成 true），實車由人手動 `ros2 param set /race_controller start true`。
