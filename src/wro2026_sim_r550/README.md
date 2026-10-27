# wro2026_sim_r550：R550 的 Gazebo 模擬（只用在模擬）

2026-10-20 開始，**暫時**放在 Orin 上跑（CLAUDE.md）。目的：等三用電表、Orin 上車的期間，先在模擬裡把建圖和 Nav2 的參數調好。

## 跟實車一樣的地方

| 項目 | 模擬 | 來源 |
|---|---|---|
| 車身 | 後軸中心往前 0.24 m、往後 0.031 m，寬 0.12 m（含輪子 0.19 m） | 實量 2026-10-20 |
| 阿克曼 | 軸距 0.144、輪距 0.162、輪半徑 0.0342 m；只有後輪出力；steering_limit 0.424 rad（gz 用 軸距 / sin 算，= 最小轉彎半徑 0.35 m）；外掛不設速度限制（gz 會連角速度一起限），限速全交給 CommandGate | 韌體、實量 |
| 光達 | (0.13, 0, 0.115)、480 格、10 Hz、`laser_link`、雜訊 8 mm | URDF、長時間測試 |
| IMU | 20 Hz、`imu_link`、gyro 雜訊 0.00037 rad/s、z 偏差 0.0006 rad/s | 長時間測試 |
| topic | `/odom`、`/imu/data`、`/imu/data_raw`、`/scan`、`/cmd_vel`、`/cmd_vel_out` | 跟實車一樣 |
| 安全層 | `/cmd_vel` 經過**同一個 CommandGate**（逾時、限速、不原地轉） | `wro2026_base_r550` |
| ZUPT、EKF | 同一個 `zupt.py`、同一份 `ekf.yaml` | |
| TF | 同一份 URDF（`robot_state_publisher`） | `wro2026_description` |

**不一樣**：車重 2.0 kg、車身高度是估計的（UNKNOWN）；Orin 沒有畫出來；里程計比例不用校正（模擬的輪子就是真的半徑）；電池固定 12 V。

## 結構

```
/cmd_vel ─▶ sim_base_node（CommandGate）─▶ /sim/cmd_vel ─▶ Gazebo 阿克曼外掛
Gazebo ─▶ /sim/odom（輪子算的）、/sim/imu ─▶ sim_sensors_node ─▶ /odom、/imu/data、/imu/data_raw
Gazebo ─▶ /scan（GPU 光達）
Gazebo ─▶ /sim/ground_truth（真實位置，只給 drive_test_node 對答案，不進控制）
```

## 用法（Orin，ssh）

```bash
export ROS_DOMAIN_ID=42                                   # 和實車的 ROS 隔開
ros2 launch wro2026_sim_r550 sim.launch.py                # Gazebo（無畫面）+ EKF + Foxglove
ros2 run wro2026_base_r550 teleop_node                    # 另一個終端機，也要先 export ROS_DOMAIN_ID=42
bash ~/racecar_ws/tools/orin_sim_check.sh                 # 自動檢查（Claude Code 可以跑）
```

畫面用 Foxglove 看（ws://<Orin>:8765）：`/scan`、TF、`/odometry/filtered`。

繞圈、建圖、Nav2（sim.launch.py 開著時，每個指令都在另一個終端機、先 `export ROS_DOMAIN_ID=42`）：

```bash
ros2 run wro2026_sim_r550 lap_node --ros-args -p use_sim_time:=true -p laps:=2      # pure pursuit 沿中心線
ros2 launch wro2026_bringup slam.launch.py use_sim_time:=true                       # 邊跑邊建圖
ros2 run nav2_map_server map_saver_cli -f ~/maps/field --ros-args -p save_map_timeout:=10.0
ros2 launch wro2026_bringup nav2.launch.py use_sim_time:=true \
    map:=$(ros2 pkg prefix wro2026_sim_r550)/share/wro2026_sim_r550/maps/field_sim.yaml
ros2 run wro2026_sim_r550 nav_lap_node --ros-args -p use_sim_time:=true -p laps:=2  # Nav2 繞圈 + 檢查
```

2026-10-21 結果（Orin）：lap_node 2 圈偏離中心線最大 0.044 m；建圖所有格子離真牆 < 0.04 m（`maps/field_sim`，地圖座標 = 場地座標）；Nav2 2 圈 0 次恢復、偏離最大 0.081 m。

### 2026-10-27 追蹤精度 campaign 加的選項（預設值 = 原本的行為）

報告：`docs/real/tasks/2026-10-27_sim_tracking_campaign.md`。

| 哪裡 | 參數 | 做什麼 |
|---|---|---|
| sim.launch.py | `seed:=N` | gz 亂數種子（IMU 雜訊、gyro 偏差正負號），可重現 |
| sim.launch.py | `steer_p_gain:=G` | 外掛轉向增益：轉向角是一階延遲 τ ≈ 1/G。**沒設 = gz 預設 1.0 → τ ≈ 1 s**，比 RC 伺服慢很多；實車值 UNKNOWN，量到後再設 |
| sim.launch.py | `obstacles:="x,y;x,y"`、`obstacle_size` | 場上放靜態方塊（預設 5×5×20 cm：WRO 號誌的底面積，高度讓光達看得到） |
| sim.launch.py | `gyro_scale`、`odom_vx_scale` | 模擬感測器校正（跟實車的 vx_scale / gyro 校正同一件事）；量到的值 1.005 / 0.981 |
| sim.launch.py | `ekf_params:=FILE` | 換一份 EKF 設定做模擬實驗（預設 = 實車的 ekf.yaml） |
| lap_node | `controller` lag_ff / stanley / rpp | lag_ff：模型化轉向延遲（`steer_tau`、`steer_delay`）並提前補償；stanley：前軸追線；rpp：像 Nav2 的 Regulated Pure Pursuit |
| lap_node | `transition`、`inset` | 路線：彎道兩端 clothoid 長度；直道斜一點讓出彎內收（對付前懸外甩） |
| lap_node | `obstacles:=[x1,y1,...]`、`pass_inside`、`pass_outside` | 有障礙物的那一邊整條線移到內側 / 外側（預設 0.38 / 0.60，零誤差時離障礙物 7.2 / 6.9 cm） |
| lap_node | `pose_source:=truth` | **診斷用**：控制器吃 Gazebo 真值（只有模擬有），用來把定位誤差和控制誤差分開 |
| nav_lap_node | `goals` corners / straights / path | Nav2 的目標：4 個彎後點（原本）/ 8 個直道端點 / 直接給參考路線（FollowPath，只用 RPP） |

離線工具（Windows 也能跑，不需要 ROS）：`tools/analysis/lap_sim.py`（同一份控制程式 + CommandGate + 轉向延遲模型，1 秒跑 2 圈）、`lap_metrics.py`（評分）、`path_check.py`（零誤差離牆）。

WSL 有畫面看（跟 Orin 分開，domain 43）：`bash /mnt/c/Users/chaon/ros2_racecar/tools/wsl/sim_r550_view.sh [drive|lap|nav]`，測完 5 秒自己關。

## 安全

- 實車的 `base_node`、`feedback_node`、`scan_node` 在跑時，`sim.launch.py` 會拒絕啟動。
- 只有 `/sim/cmd_vel` 會送進 Gazebo；`/cmd_vel` 一定先經過 CommandGate。
- `ROS_DOMAIN_ID=42`：模擬的 `/cmd_vel` 不會被實車的 `base_node`（預設 domain 0）收到。
- `/sim/ground_truth` 只給檢查程式用。
