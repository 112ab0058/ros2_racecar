# wro2026_sim_r550：R550 的 Gazebo 模擬（只用在模擬）

2026-10-20 開始，**暫時**放在 Orin 上跑（CLAUDE.md）。目的：等三用電表、Orin 上車的期間，先在模擬裡把建圖和 Nav2 的參數調好。

## 跟實車一樣的地方

| 項目 | 模擬 | 來源 |
|---|---|---|
| 車身 | 後軸中心往前 0.24 m、往後 0.031 m，寬 0.12 m（含輪子 0.19 m） | 實量 2026-10-20 |
| 阿克曼 | 軸距 0.144、輪距 0.162、輪半徑 0.0342 m；只有後輪出力；中心轉角上限 0.39 rad（= 最小轉彎半徑 0.35 m） | 韌體、實量 |
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

## 安全

- 實車的 `base_node`、`feedback_node`、`scan_node` 在跑時，`sim.launch.py` 會拒絕啟動。
- 只有 `/sim/cmd_vel` 會送進 Gazebo；`/cmd_vel` 一定先經過 CommandGate。
- `ROS_DOMAIN_ID=42`：模擬的 `/cmd_vel` 不會被實車的 `base_node`（預設 domain 0）收到。
- `/sim/ground_truth` 只給檢查程式用。
