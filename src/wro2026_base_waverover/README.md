# wro2026_base_waverover

Waveshare WAVE ROVER 的底盤 driver（過渡測試車）。由 Jetson 上已驗證的 `wave_rover_driver`（本機 git `543a4fc`）搬進來，功能與參數保留，結構拆成三層：

| 層 | 檔案 | 內容 | 依賴 |
|---|---|---|---|
| 純 Python 核心 | `wro2026_base_waverover/core.py` | 差速運動學、限速、JSON 編碼、`/cmd_vel` timeout、回授遺失偵測、電池門檻、odom 積分 | 只有標準函式庫 |
| Transport | `wro2026_base_waverover/transport.py` | `HttpTransport`、`SerialTransport`、測試用的 `FakeTransport` | `python3-serial`（只有 serial 需要） |
| ROS 2 node | `wro2026_base_waverover/driver_node.py` | 參數、publisher / subscriber、worker thread | rclpy 等 |

換車（例如之後的 Ackermann 比賽車）時只換這個 package；上層只用 `/cmd_vel`、`/odom`、TF、`/imu/data`、`/battery`。

## 執行

```bash
# Jetson，HTTP（Wi-Fi）
ros2 launch wro2026_base_waverover bringup.launch.py transport:=http host:=192.168.0.16
# Jetson，UART（/dev/ttyTHS1）
ros2 launch wro2026_base_waverover bringup.launch.py
```

完整機器人（driver + robot_state_publisher）用 `ros2 launch wro2026_real bringup.launch.py`。
參數全部在 `config/wave_rover.yaml`；launch 參數 `<name>:=<value>` 只覆寫，空值代表用 YAML 的值。
**舊的 `wave_rover_driver` 和這個 driver 不能同時跑**：這個 driver 和 `test_motion` 啟動時會檢查，發現舊的或另一個新的就拒絕啟動。舊 driver 本身不會檢查，所以先開新的再開舊的時擋不住，要自己確認。

## Topic

| Topic | 型別 | 說明 |
|---|---|---|
| `/cmd_vel`（訂閱） | `geometry_msgs/Twist` | `linear.x`、`angular.z` 直接當 PWM 比例做差速換算（**不是 m/s**），每輪限制在 `max_speed`；NaN / inf 當成停止 |
| `/battery` | `sensor_msgs/BatteryState` | T:130 → T:1001 的 `v`，只有電壓 |
| `/imu/data` | `sensor_msgs/Imu` | T:126 → T:1002，covariance 全 0（未知） |
| `/odom`、TF `odom → base_link` | `nav_msgs/Odometry` | **低品質 dead reckoning**，見下 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK / WARN / ERROR 與原因 |
| `/wave_rover/feedback` | `std_msgs/String` | 每一行原始回覆 |
| `/wave_rover/sent_cmd` | `std_msgs/String` | 每一筆送出的 T:1（`t`、`dt`、`ok`、`cmd`、`gate`） |

## 安全行為

| 條件 | 行為 |
|---|---|
| 啟動 | 只送 `{"T":1,"L":0,"R":0}`（零一律編成整數 0） |
| `/cmd_vel` 含 NaN / inf | 當成停止，log warn |
| 指令一直在變（HTTP） | 每送一次輪速就讓出一次給到期的回授詢問，T:130 不會被擠掉；新的停止指令永遠立刻送 |
| T:1 送失敗 | 0.1 s 後重送（不是等 `cmd_period`） |
| `/cmd_vel` 超過 `cmd_timeout`（0.3 s）沒來 | 送停止 |
| 連續 `feedback_timeout`（2 s）沒收到 T:130 回應 | log error、立刻送停止、拒絕非零指令、`/diagnostics` ERROR；回授恢復後才放行。收到第一筆回應之前也拒絕非零指令 |
| 電池 < `low_battery_voltage`（11.0 V） | `/diagnostics` WARN、log warn |
| 電池 < `critical_battery_voltage`（10.5 V） | 拒絕非零指令、`/diagnostics` ERROR |
| 關閉（SIGINT） | 送停止，最多試 3 次，log `shutdown stop sent: ok/FAILED` |
| ESP32 本身 | 約 3 s 沒收到指令自動停 |

`max_speed:=0.0` 時任何 `/cmd_vel` 都只會送出停止，可用來做不會動輪子的測試。

## Odometry（低品質，不可作為 Nav2 的主要來源）

WAVE ROVER 沒有 encoder。

- 線速度 = 實際送出成功的左右輪指令平均 × `pwm_to_mps`。**`pwm_to_mps` 實測值 UNKNOWN，預設 0：odom 只會轉向、不會位移。** 量測方法見 `docs/real/RUNBOOK.md`。
- 角速度 = IMU `gz`（`imu_yaw_rate_sign` 修正方向，方向待實測）；IMU 超過 `imu_timeout` 沒更新時當 0。
- 送出的指令是零時（`odom_hold_when_stopped`），位置和角度都不動，避免陀螺儀偏差讓 odom 漂移。
- covariance 刻意設大（x/y 100 m²、yaw 10 rad²）。
- `publish_odom_tf:=false` 可以關掉 TF（例如之後改由 robot_localization 發佈）。

## 工具

```bash
ros2 run wro2026_base_waverover check_serial            # UART 唯讀檢查，不寫入任何資料
ros2 run wro2026_base_waverover test_motion --startup-only --transport serial   # 不動輪子
ros2 run wro2026_base_waverover test_motion --transport serial --speed 0.05      # 會動輪子：必須架空
```

`test_motion` 會自己啟動 driver（用 `config/wave_rover.yaml` 再加上命令列參數），log 寫到 `~/wave_rover_logs/`。

## 測試

不需要硬體也不需要 ROS：

```bash
cd src/wro2026_base_waverover && python3 -m pytest     # 任何有 pytest 的機器
colcon test --packages-select wro2026_base_waverover && colcon test-result --verbose   # Jetson（另外跑 flake8 / pep257）
```

## UNKNOWN（待實測）

`track_width`（預設 0.125 m 是韌體常數）、輪徑、`pwm_to_mps`、IMU 軸向與 `base_link` 的關係、0.05 PWM 時輪子是否真的轉動。
