# WRO 2026 - 當前狀態快照
更新時間：2026-07-05

這份文件是目前 160 x 160 cm 專題場地版本的狀態整理。

- 實際執行工作區：`/home/ray/ros2_ws`
- GitHub repo 工作區：`/home/ray/ros2_racecar`
- GitHub remote：`https://github.com/112ab0058/ros2_racecar.git`

## 現在做到哪

目前主線已從舊 3.2 m WRO 模擬場地收斂到 1600 x 1600 mm 專題場地。

目前狀態：

- Step 5：相機 HSV 橘線/藍線偵測已完成。
- Step 6：LiDAR 牆面置中、ground-truth odom 控制迴圈已完成。
- Step 6/7：用 `/line_detection` 觸發轉彎的狀態機已完成。
- 160 cm 場地：高速直線區、上方連續避障區、右側停車/靠站區已納入 Gazebo world。
- 避障第一版：LiDAR gap avoidance 已接到 `line_follower.py`。
- Step 7：YOLO 紅/綠柱偵測尚未完成，先不讓 AI 直接控制車。
- 停車、CAN/MCU、encoder/IMU、Nav2 實車整合尚未完成，但已放入架構文件。

目前官方完整模擬啟動方式：

```bash
ros2 launch wro2026_sim race.launch.py
```

這個 launch 會用 server-only Gazebo 啟動正式 160 cm world、ROS-GZ bridge、ground-truth odom、line detector、line follower。

## 已完成內容

### 模擬與世界檔

- 正式場地在 `wro2026_sim/worlds/wro2026_field.sdf`。
- 場地尺寸：`1600 x 1600 mm`。
- 中央分隔島：`400 x 400 mm`。
- 下方高速直線區、上方連續避障區、右側停車/靠站區已建入 world。
- 橘/藍線使用舊 3.2 m world 的對角線配置縮放到 160 cm，不是橫直十字線。
- 右側停車/靠站隔板目前是 visual-only，不讓它卡住主賽道。
- 舊的 `open_wide`、`open_narrow`、`obstacle_wide` 世界檔已移除。
- 模擬車型使用 `turtlebot3_burger_cam`。
- 模擬牆高維持 `0.30 m`，確保 TurtleBot3 LiDAR 掃得到。
- Physics `max_step_size` 已從舊版 `0.001` 放寬為 `0.005`，讓 WSL 更順。
- `models/turtlebot3_burger_cam` 已納入 repo，不再依賴 WSL 裡的散落模型。
- 相機維持 `320 x 240 @ 10 Hz`，LiDAR 調為 `8 Hz`，camera visualize 關閉。

### Odometry

- `gazebo_ground_truth_odom.py` 從 Gazebo `/world/wro2026/dynamic_pose/info` 讀真實位置。
- 發布 `/odom`。
- 可發布 `odom -> base_footprint` TF。
- `race.launch.py` 傳入 `model_name=turtlebot3_burger_cam`。

### 橘線/藍線偵測

- `line_detector.py` 訂閱 `/camera/image_raw`。
- 發布 `/line_detection`。
- debug image 預設關閉，避免 WSL/Gazebo 主測試增加影像疊圖負擔。
- HSV 範圍：
  - 橘色：`[8, 150, 150]` 到 `[30, 255, 255]`
  - 藍色：`[100, 150, 50]` 到 `[130, 255, 255]`
- 面積門檻：`200`
- 觸發門檻：`1000`

`/line_detection` topic 格式：

```text
data[0] orange_detected
data[1] orange_cx
data[2] orange_area
data[3] orange_trigger
data[4] blue_detected
data[5] blue_cx
data[6] blue_area
data[7] blue_trigger
data[8] direction
```

### Line-triggered controller

目前主控制器是：

```text
~/ros2_ws/src/wro2026_sim/scripts/line_follower.py
```

主要狀態：

```text
STRAIGHT -> APPROACH -> TURN -> ALIGN -> POST_TURN -> STRAIGHT
```

新增避障狀態：

```text
AVOID_OBSTACLE -> RETURN_TO_LINE -> STRAIGHT
```

另有 `RECOVER` / `RECOVER_FAILED` 處理距離過近。

目前重要行為：

- 正常轉彎由 `/line_detection` 觸發。
- 橘線 trigger 代表右轉。
- 藍線 trigger 代表左轉。
- 前方 LiDAR 距離做緊急保護、recovery、以及上方避障區 gap avoidance。
- 直線時用左右側與斜前方 LiDAR 做牆面置中。
- 轉彎完成用 odom yaw 判斷，目標是 90 度。
- 下方高速直線區提高速度。
- 上方避障區降速，但除非太近不主動停車。

目前關鍵參數：

```text
STRAIGHT_SPEED = 0.125
FAST_STRAIGHT_SPEED = 0.180
OBSTACLE_ZONE_SPEED = 0.090
APPROACH_SPEED = 0.080
ALIGN_SPEED = 0.070
POST_TURN_SPEED = 0.095
TURN_LINEAR_SPEED = 0.045
MAX_TURN_ANGULAR = 0.58
MIN_TURN_ANGULAR = 0.20
TURN_KP = 0.95
LINE_APPROACH_DIST = 0.10
AVOID_TRIGGER_DIST = 0.46
AVOID_CLEAR_DIST = 0.62
```

## Launch 檔狀態

### 官方 race launch

```text
~/ros2_ws/src/wro2026_sim/launch/race.launch.py
```

會啟動：

- `gz sim -r -s wro2026_field.sdf`
- `/cmd_vel`、`/scan`、`/clock`、`/camera/image_raw`、`/camera/camera_info` bridge
- `gazebo_ground_truth_odom.py`
- `line_detector.py`
- `line_follower.py`

### Mapping launch

```text
~/ros2_ws/src/wro2026_sim/launch/mapping.launch.py
```

只在要建圖時使用。不要和 race controller 混在一起跑。

### Future physical bringup

```text
~/ros2_ws/src/wro2026_sim/launch/physical_bringup.launch.py
```

目前是 placeholder，用來保留之後 encoder / IMU / CAN / MCU 實車整合入口。

## 架構文件

- `docs/SYSTEM_ARCHITECTURE.md`：老師要求的 encoder / MCU / CAN / ROS2 / SLAM / Nav2 / Foxglove 架構。
- `docs/FIELD_PLAN_160CM.md`：160 x 160 cm 場地尺寸、材料、用途。
- `docs/GAZEBO_PERFORMANCE.md`：WSL/Gazebo 效能設定與測試方式。

## 關鍵檔案

| 用途 | WSL 路徑 |
|---|---|
| 官方完整測試 launch | `~/ros2_ws/src/wro2026_sim/launch/race.launch.py` |
| Mapping launch | `~/ros2_ws/src/wro2026_sim/launch/mapping.launch.py` |
| 實車預留入口 | `~/ros2_ws/src/wro2026_sim/launch/physical_bringup.launch.py` |
| 主控制器 | `~/ros2_ws/src/wro2026_sim/scripts/line_follower.py` |
| 橘/藍線偵測 | `~/ros2_ws/src/wro2026_sim/scripts/line_detector.py` |
| ground-truth odom | `~/ros2_ws/src/wro2026_sim/scripts/gazebo_ground_truth_odom.py` |
| 主世界檔 | `~/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf` |
| Gazebo 車體模型 | `~/ros2_ws/src/wro2026_sim/models/turtlebot3_burger_cam` |
| GitHub repo copy | `~/ros2_racecar` |

## 不要誤動

- 不要把模擬牆高改回真實 0.1 m，TurtleBot3 LiDAR 會掃不到。
- 不要移除 `gazebo_ground_truth_odom.py`，它是目前模擬穩定 odom 來源。
- 不要在 `race.launch.py` 裡啟動 SLAM/Nav2，效能會拖慢主測試。
- 不要假設實車 HSV 會和 Gazebo 一樣，實車相機要重新調。

## 下一步

1. 跑 `race.launch.py`，確認 160 cm world 能順暢啟動。
2. 無障礙物先跑 3 圈，確認橘/藍線 trigger 和轉彎距離。
3. 測 1 個、3 個障礙物的 `AVOID_OBSTACLE` / `RETURN_TO_LINE`。
4. 如果切角或撞牆，優先調 `LINE_APPROACH_DIST`、`TURN_LINEAR_SPEED`、`TURN_KP`。
5. 加入停車偵測與停車動作。
6. Orin Nano 實車部署要獨立整理，不要直接把 WSL 模擬參數照搬。
