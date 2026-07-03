# Workspace Map - WSL 與 GitHub 對照

更新時間：2026-07-03

## 程式碼在哪裡找到

WSL 檔案系統來源：

```text
C:\Users\chaon\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu_79rhkp1fndgsc\LocalState\ext4.vhdx
```

WSL 裡的 Linux 使用者：

```text
/home/ray
```

## 主要資料夾

### `/home/ray/ros2_ws`

這是 ROS2 實際執行工作區。

目前 launch 檔和程式裡大多假設路徑是：

```text
~/ros2_ws/src/wro2026_sim
```

要跑 Gazebo 模擬時，以這裡為準。

### `/home/ray/ros2_racecar`

這是 GitHub repo 工作區。

Remote：

```text
https://github.com/112ab0058/ros2_racecar.git
```

要 commit / push 時，以這裡為準。

### `/home/ray/learning_tf2_py`

這看起來是較早的 ROS2 TF tutorial package，不是目前 WRO 主專案。

### `/home/ray/turtle_bridge.py`、`/home/ray/start_turtle.sh`、`/home/ray/go_carrot.sh`

這些是較早的 turtle / bridge 輔助腳本，不是目前 WRO Step 6/7 的主流程。

## 主專案 package

### `wro2026_sim`

位置：

```text
~/ros2_ws/src/wro2026_sim
```

用途：

- Gazebo world
- launch file
- 橘/藍線偵測
- line following / race controller
- ground-truth odometry
- map 與 SLAM config

重要檔案：

```text
launch/wro2026_step6_lidar.launch.py
launch/wro2026_step6_control.launch.py
launch/wro2026_sim.launch.py
scripts/line_follower.py
scripts/line_detector.py
scripts/gazebo_ground_truth_odom.py
worlds/wro2026_field.sdf
config/slam_params.yaml
```

### `wro2026_mapper`

位置：

```text
~/ros2_ws/src/wro2026_mapper
```

用途：

- 獨立 lawn-mower mapping tool
- 會發布 `/cmd_vel`

不要和 `line_follower.py` 同時跑。

### `my_racecar_pkg`

位置：

```text
~/ros2_racecar/src/my_racecar_pkg
```

用途：

- 較早的 ROS2 tutorial / racecar package
- 目前不是官方 WRO Step 6/7 workflow

除非之後實車整合明確要用，否則先視為 legacy。

## 要改什麼檔案

| 任務 | 改這裡 |
|---|---|
| 調速度或轉彎行為 | `~/ros2_ws/src/wro2026_sim/scripts/line_follower.py` |
| 調橘/藍 HSV 偵測 | `~/ros2_ws/src/wro2026_sim/scripts/line_detector.py` |
| 改 Gazebo odom model 來源 | `~/ros2_ws/src/wro2026_sim/scripts/gazebo_ground_truth_odom.py` |
| 加 camera / LiDAR bridge topic | `~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_lidar.launch.py` |
| 改場地幾何 | `~/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf` |
| 改 SLAM 參數 | `~/ros2_ws/src/wro2026_sim/config/slam_params.yaml` |
| 更新目前狀態 | `~/ros2_racecar/docs/CURRENT_STATE.md` |
| GitHub commit / push | `~/ros2_racecar` |

## 建議同步流程

WSL 恢復正常後，先看 repo 狀態：

```bash
cd ~/ros2_racecar
git status
git log --oneline --decorate --max-count 8
```

只同步真正主專案 package：

```bash
rsync -av --delete ~/ros2_ws/src/wro2026_sim/ ~/ros2_racecar/src/wro2026_sim/
rsync -av --delete ~/ros2_ws/src/wro2026_mapper/ ~/ros2_racecar/src/wro2026_mapper/
```

同步後檢查：

```bash
cd ~/ros2_racecar
git status
git diff --stat
git diff -- src/wro2026_sim src/wro2026_mapper docs
```

只提交主專案與文件：

```bash
git add src/wro2026_sim src/wro2026_mapper docs
git commit -m "Update Step 6 line-triggered race control state"
git push origin main
```

## 目前 GitHub 整合注意事項

恢復出的 repo 顯示：

```text
main ahead of origin/main by 1 commit
188d150 Organize mapper and mark legacy SLAM launch
```

push 前先確認 GitHub 有沒有別台機器的新 commit：

```bash
git fetch origin
git status --short --branch
```

如果只顯示 `ahead`，可以直接 push。

如果同時顯示 `ahead` 和 `behind`，先 rebase：

```bash
git pull --rebase origin main
git push origin main
```
