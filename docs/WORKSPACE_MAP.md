# Workspace Map - WSL 與 GitHub 對照

更新時間：2026-07-05

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

ROS2 實際執行工作區。Gazebo launch 內的路徑以這裡為準：

```text
~/ros2_ws/src/wro2026_sim
```

### `/home/ray/ros2_racecar`

GitHub repo 工作區。

Remote：

```text
https://github.com/112ab0058/ros2_racecar.git
```

## 主專案 package

### `wro2026_sim`

目前主線 package。

重要檔案：

```text
launch/race.launch.py
launch/mapping.launch.py
launch/physical_bringup.launch.py
scripts/line_follower.py
scripts/line_detector.py
scripts/gazebo_ground_truth_odom.py
models/turtlebot3_burger_cam
worlds/wro2026_field.sdf
config/slam_params.yaml
```

### Future packages

目前用文件規劃，不急著建立可編譯 package：

```text
wro2026_bringup
wro2026_can_bridge
wro2026_description
wro2026_navigation
```

## 要改什麼檔案

| 任務 | 改這裡 |
|---|---|
| 調速度、轉彎、避障 | `~/ros2_ws/src/wro2026_sim/scripts/line_follower.py` |
| 調橘/藍 HSV 偵測 | `~/ros2_ws/src/wro2026_sim/scripts/line_detector.py` |
| 改 Gazebo odom model 來源 | `~/ros2_ws/src/wro2026_sim/scripts/gazebo_ground_truth_odom.py` |
| 加 camera / LiDAR bridge topic | `~/ros2_ws/src/wro2026_sim/launch/race.launch.py` |
| 改場地幾何 | `~/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf` |
| 改 SLAM 參數 | `~/ros2_ws/src/wro2026_sim/config/slam_params.yaml` |
| 更新目前狀態 | `~/ros2_racecar/docs/CURRENT_STATE.md` |
| 場地與材料規劃 | `~/ros2_racecar/docs/FIELD_PLAN_160CM.md` |
| 系統架構 | `~/ros2_racecar/docs/SYSTEM_ARCHITECTURE.md` |
| Gazebo 效能 | `~/ros2_racecar/docs/GAZEBO_PERFORMANCE.md` |
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
```

同步後檢查：

```bash
cd ~/ros2_racecar
git status
git diff --stat
git diff -- src/wro2026_sim docs README.md
```

提交主專案與文件：

```bash
git add src/wro2026_sim docs README.md
git commit -m "Update 160cm Gazebo race field"
git push origin main
```
