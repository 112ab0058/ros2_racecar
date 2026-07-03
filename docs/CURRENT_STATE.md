# WRO 2026 - 當前狀態快照
更新時間：2026-07-03

這份文件是從 WSL 磁碟恢復後整理出的目前狀態。

- 實際執行工作區：`/home/ray/ros2_ws`
- GitHub repo 工作區：`/home/ray/ros2_racecar`
- GitHub remote：`https://github.com/112ab0058/ros2_racecar.git`

## 現在做到哪

目前已經不是舊版 2026-05-19 的 Step 5 狀態，實際程式已經推進到 Step 6/7 交界。

目前實作狀態：

- Step 5：相機 HSV 橘線/藍線偵測已完成。
- Step 6：LiDAR 牆面置中、ground-truth odom 控制迴圈已完成。
- Step 6/7：用 `/line_detection` 觸發轉彎的狀態機已完成。
- Step 7：YOLO 紅/綠柱偵測尚未完成。
- 停車與 Orin 實車整合尚未完成。

目前官方完整模擬啟動方式：

```bash
ros2 launch ~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_lidar.launch.py
```

這個 launch 會啟動 Gazebo、ROS-GZ bridge、ground-truth odom、line detector、line follower。

## 已完成內容

### 模擬與世界檔

- Gazebo Harmonic 的 WRO 2026 場地在 `wro2026_sim/worlds/wro2026_field.sdf`。
- 另有 `open_wide`、`open_narrow`、`obstacle_wide` 世界檔。
- 模擬車型使用 `turtlebot3_burger_cam`。
- Gazebo resource path 已包含本地 models 和 TurtleBot3 Gazebo models。
- 模擬牆高維持 LiDAR 掃得到的高度，不要改回實際 0.1 m。

### Odometry

- `gazebo_ground_truth_odom.py` 從 Gazebo `/world/wro2026/dynamic_pose/info` 讀真實位置。
- 發布 `/odom`。
- 可發布 `odom -> base_footprint` TF。
- 官方 Step 6/7 launch 傳入 `model_name=turtlebot3_burger_cam`。

### 橘線/藍線偵測

- `line_detector.py` 訂閱 `/camera/image_raw`。
- 發布 `/line_detection`。
- debug 模式且有 `cv_bridge` 時，會發布 `/line_detection/debug_image`。
- HSV 範圍：
  - 橘色：`[8, 150, 150]` 到 `[30, 255, 255]`
  - 藍色：`[100, 150, 50]` 到 `[130, 255, 255]`
- 面積門檻：`200`
- 觸發門檻：`1000`

`/line_detection` topic 格式：

```text
data[0] orange_detected   1=有 0=無
data[1] orange_cx         重心 x，無則 -1
data[2] orange_area       面積
data[3] orange_trigger    是否超過觸發門檻
data[4] blue_detected     1=有 0=無
data[5] blue_cx           重心 x，無則 -1
data[6] blue_area         面積
data[7] blue_trigger      是否超過觸發門檻
data[8] direction         1=順時針 -1=逆時針 0=未知
```

### Line-triggered controller

目前主控制器是：

```text
~/ros2_ws/src/wro2026_sim/scripts/line_follower.py
```

它實作的狀態機：

```text
STRAIGHT -> APPROACH -> TURN -> ALIGN -> POST_TURN -> STRAIGHT
```

另有 recovery 狀態處理前方太近的緊急狀況。

目前重要行為：

- 正常轉彎由 `/line_detection` 觸發，不再由前方牆距離決定。
- 橘線 trigger 代表右轉。
- 藍線 trigger 代表左轉。
- 前方 LiDAR 距離只做緊急保護與 recovery。
- 直線時用左右側與斜前方 LiDAR 做牆面置中。
- 轉彎完成用 odom yaw 判斷，目標是 90 度。
- 目前轉彎不是原地旋轉，轉彎時有前進速度。

目前關鍵參數：

```text
STRAIGHT_SPEED = 0.170
APPROACH_SPEED = 0.095
ALIGN_SPEED = 0.085
POST_TURN_SPEED = 0.120
TURN_LINEAR_SPEED = 0.060
MAX_TURN_ANGULAR = 0.62
MIN_TURN_ANGULAR = 0.20
TURN_KP = 1.05
LINE_TRIGGER_COOLDOWN_SEC = 2.0
LINE_TRIGGER_LOCKOUT_DIST = 0.45
LINE_APPROACH_DIST = 0.14
TURN_YAW_TOLERANCE_RAD = 5 deg
TURN_MIN_PROGRESS_RAD = 76 deg
```

### Mapper

`wro2026_mapper` 已拆成獨立 mapping tool。

它也會發布 `/cmd_vel`，所以不能和 `line_follower.py` 同時跑。

單獨執行：

```bash
ros2 run wro2026_mapper wro2026_mapper
```

## Launch 檔狀態

### 官方 Step 6/7 完整測試

```text
~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_lidar.launch.py
```

會啟動：

- `gz sim -r wro2026_field.sdf`
- `/cmd_vel`、`/scan`、`/clock`、`/camera/image_raw`、`/camera/camera_info` bridge
- `gazebo_ground_truth_odom.py`
- `line_detector.py`
- `line_follower.py`

### LiDAR/odom debug baseline

```text
~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_control.launch.py
```

這個 launch 不會啟動 camera bridge，也不會啟動 `line_detector.py`。
只適合拿來做 LiDAR/odom debug baseline，不是目前官方完整測試入口。

### SLAM / mapping workflow

```text
~/ros2_ws/src/wro2026_sim/launch/wro2026_sim.launch.py
```

這是模擬與 SLAM/mapping 流程，不是目前 Step 6/7 line-triggered race workflow。

## GitHub 狀態

Repo：

```text
/home/ray/ros2_racecar
```

Remote：

```text
origin https://github.com/112ab0058/ros2_racecar.git
```

從恢復出的 WSL repo 觀察到：

```text
main 比 origin/main 多 1 個 commit
本地最新 commit：188d150 Organize mapper and mark legacy SLAM launch
origin/main：1658de0 Step 6: stabilize lidar control loop
```

目前還有未提交修改，主要是：

- `src/wro2026_sim/scripts/line_follower.py`
- `src/wro2026_sim/launch/wro2026_step6_control.launch.py`
- 一批舊的 `my_racecar_pkg` tutorial 檔案，看起來多半是抽檔造成的權限或換行雜訊，不建議放進下一個 commit。

建議 GitHub 整合方式：

1. 真正主專案只整理 `src/wro2026_sim`、`src/wro2026_mapper`、`docs`。
2. 舊的 `my_racecar_pkg` 不要混進下一次 commit，除非之後明確要用於實車。
3. 下一個 commit 建議包含 Step 6/7 line-triggered controller 與文件更新。
4. 確認 GitHub 沒有別台機器的新 commit 後，再 push。

## 關鍵檔案

| 用途 | WSL 路徑 |
|---|---|
| 官方完整測試 launch | `~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_lidar.launch.py` |
| LiDAR/odom debug launch | `~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_control.launch.py` |
| 主控制器 | `~/ros2_ws/src/wro2026_sim/scripts/line_follower.py` |
| 橘/藍線偵測 | `~/ros2_ws/src/wro2026_sim/scripts/line_detector.py` |
| ground-truth odom | `~/ros2_ws/src/wro2026_sim/scripts/gazebo_ground_truth_odom.py` |
| 主世界檔 | `~/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf` |
| mapper package | `~/ros2_ws/src/wro2026_mapper` |
| GitHub repo copy | `~/ros2_racecar` |

## 不要誤動

- 不要把模擬牆高改回真實 0.1 m，LiDAR 會掃不到。
- 不要移除 `gazebo_ground_truth_odom.py`，它是目前模擬穩定 odom 來源。
- 不要同時跑 `wro2026_mapper` 和 `line_follower.py`，兩者都會發 `/cmd_vel`。
- 不要把 `wro2026_step6_control.launch.py` 當完整測試入口，它沒有啟動相機線偵測。
- 不要假設實車 HSV 會和 Gazebo 一樣，實車相機要重新調。

## 下一步

1. 跑 `wro2026_step6_lidar.launch.py`，確認 line-triggered turns 能完整跑一圈。
2. 如果切角或撞牆，優先調 `LINE_APPROACH_DIST`、`TURN_LINEAR_SPEED`、`TURN_KP`。
3. 加入紅/綠柱偵測。
4. 把 obstacle avoidance 狀態整合進 controller。
5. 加入停車偵測與停車動作。
6. Orin Nano 實車部署要獨立整理，不要直接把 WSL 模擬參數照搬。
