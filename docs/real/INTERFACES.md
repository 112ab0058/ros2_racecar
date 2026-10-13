# 底盤 package 的介面合約

更新時間：2026-10-14。每個底盤 package（現在的 `wro2026_base_waverover`、之後的 `wro2026_base_r550`）都必須提供下面的 topic 和安全行為；上層（`wro2026_real`、`wro2026_control`）只依賴這份合約，換車時不用改。單位與座標照 ROS 慣例（REP-103：m、s、rad，x 前、y 左、z 上）。

## 1. 底盤必須提供的 topic

| Topic | 方向 | 型別 | 單位 / 內容 | frame | 頻率 |
|---|---|---|---|---|---|
| `/cmd_vel` | 底盤訂閱 | `geometry_msgs/Twist` | `linear.x` m/s、`angular.z` rad/s；其他欄位忽略；NaN / inf 當成停止 | `base_link` | 由上層決定（控制器 20 Hz） |
| `/odom` | 底盤發佈 | `nav_msgs/Odometry` | 位置 m、姿態四元數、速度 m/s / rad/s；covariance 要反映實際品質 | `odom` → child `base_link` | ≥ 20 Hz |
| TF `odom → base_link` | 底盤發佈 | `tf2_msgs/TFMessage` | 和 `/odom` 相同；改由 `robot_localization` 發佈時要能關掉（`publish_odom_tf`） | — | 同 `/odom` |
| `/imu/data` | 底盤發佈 | `sensor_msgs/Imu` | 角速度 rad/s、加速度 m/s²、姿態（有的話）；不知道的 covariance 填 0、沒有的量第一個元素填 -1 | `imu_link` | ≥ 5 Hz（越高越好） |
| `/battery` | 底盤發佈 | `sensor_msgs/BatteryState` | `voltage` V；沒有的欄位填 NaN | `base_link` | ≥ 1 Hz |
| `/diagnostics` | 底盤發佈 | `diagnostic_msgs/DiagnosticArray` | OK / WARN / ERROR 與原因（回授、電池、IMU） | — | 1 Hz |

機器人描述：`wro2026_real` 的 URDF 以 `base_link` 為根，提供 `imu_link`、`camera_link`（之後加 `laser_link`），由 robot_state_publisher 發佈 static TF。

其他上層會用到、但不屬於底盤的 topic：`/scan`（光達 driver，`sensor_msgs/LaserScan`）、`/line_detection`（`wro2026_real` 的 line detector，9 個 float，見 `src/wro2026_real/README.md`）。

## 2. 底盤必須有的安全行為

| 行為 | 要求 |
|---|---|
| 啟動 | 只送停止；收到底層第一筆回授之前不送非零指令 |
| `/cmd_vel` 逾時 | 超過 `cmd_timeout`（預設 0.3 s，CLAUDE.md）沒有新指令就送停止 |
| 限速 | 參數上限，任何 `/cmd_vel` 都夾在上限內 |
| 回授遺失 | 連續 2 s 沒有底層回授：log error、送停止、拒絕非零指令、`/diagnostics` ERROR；回授恢復才放行 |
| 低電壓 | 低於警告門檻 `/diagnostics` WARN；低於危險門檻拒絕非零指令、ERROR |
| 非有限值 | NaN / inf 當成停止 |
| 關閉 | 送停止（失敗要重試，並 log 結果） |
| 單一實例 | 同一個底盤只能有一個 driver 在跑 |
| 實機 | 停止與逾時要先在架空狀態驗證，才能有在地面自主行駛的 launch |

## 3. 控制器 `wro2026_control`（`race_node`）用到的介面

所有 topic 名稱都是參數（2026-10-14 起），預設值如下。

| 參數 | 預設 | 方向 | 型別 | 用途 |
|---|---|---|---|---|
| `scan_topic` | `/scan` | 訂閱 | `LaserScan` | 車道幾何、障礙物、前方牆壁 |
| `odom_topic` | `/odom` | 訂閱 | `Odometry` | 只用 yaw 和**位置累積出的行駛距離**（不用絕對 x / y 做決策） |
| `imu_topic` | `/imu/data` | 訂閱 | `Imu` | `use_imu_yaw:=true` 時用 IMU 姿態的 yaw（Gazebo 的 launch 設成 `/imu`） |
| `line_detection_topic` | `/line_detection` | 訂閱 | `Float32MultiArray` | index 3 = 橘線觸發、index 7 = 藍線觸發 |
| `cmd_vel_topic` | `/cmd_vel` | 發佈 | `Twist` | m/s、rad/s；最後夾在 `max_linear` 0.20 m/s、`max_angular_out` 1.0 rad/s |
| `status_topic` | `/race_status` | 發佈 | `String`（JSON） | 狀態、圈數、原因 |

控制器的前提：掃描與 odom 都要在 `sensor_timeout`（0.5 s）內更新，否則它送零速；線偵測超過 `line_timeout`（0.3 s）視為沒有線；阿克曼車要設 `min_turn_radius`（每個前進指令都滿足 |ω| ≤ |v| / R）。

**注意：`race_node` 的 `start` 參數預設是 `true`**——只要 `/scan` 和 `/odom` 都在更新，它一啟動就會開始送前進指令（node 的說明寫「start 為 true 前不會動」，但預設就是 true）。在實車（R550）上一律要用 `start:=false` 啟動，確認一切正常、車輪架空後再用 `ros2 param set /race_controller start true`。建議控制器的作者把預設改成 false（這次規定不能改控制器，所以沒改）。

## 4. WAVE ROVER 目前沒做到的項目

| 項目 | 狀態 | 說明 |
|---|---|---|
| `/cmd_vel` 用 m/s | **部分** | 預設 `cmd_vel_units: pwm`（數值直接當 PWM 比例）；`cmd_vel_units: mps` 已實作，但需要實測的 `pwm_to_mps`，目前是 0，設成 mps 會拒絕啟動 |
| `/odom` 位移 | **沒有** | 沒有 encoder；`pwm_to_mps` = 0 時只轉向不位移。控制器需要行駛距離，所以**不能**在 WAVE ROVER 上跑控制器 |
| `/odom` 品質 | **低** | 指令速度 × `pwm_to_mps` + IMU gz；covariance 設很大；不可當 Nav2 主要 odom |
| `/odom` 頻率 | 有 | 20 Hz |
| TF `odom → base_link` | 有 | 可用 `publish_odom_tf` 關掉 |
| `/imu/data` | **部分** | 約 4.8 Hz（HTTP 輪詢）；covariance 全 0（未知）；gz 靜止雜訊大（標準差 5.1 deg/s）；軸向與 `base_link` 的關係未驗證 |
| `/battery` | 有 | 只有電壓 |
| `/diagnostics` | 有 | 1 Hz |
| 逾時停車 | 有 | 舊 driver 實測 0.306 s；新 driver pytest 驗證、2026-10-13 架空目視通過 |
| 限速 | 有 | `max_speed`（PWM 比例，預設 0.05） |
| 回授遺失 | 有 | pytest 驗證；沒有在實機上拔線測試 |
| 低電壓 | 有 | pytest 驗證（11.0 V WARN、10.5 V 拒絕） |
| 單一實例 | **部分** | 新 driver 會擋舊的；舊 driver 不會擋新的 |
| 啟動只送停止、第一筆回授前不送非零指令 | 有 | pytest 驗證；2026-10-13 實機只讀驗證（170 筆全是停止） |
| 非有限值當停止 | 有 | pytest 驗證（NaN / inf，pwm 與 mps 模式都是） |
| 關閉送停止 | 有 | 實機多次確認 log `shutdown stop sent: ok` |
| `mps` 模式的角速度換算 | **待校正** | 用 `track_width` 0.125 m（韌體常數，沒量過）；4WD 滑移轉向的有效輪距通常較大，要和 `pwm_to_mps` 一起實測 |
| `/scan` | **沒有** | WAVE ROVER 沒有光達 |
| `/line_detection` | **部分** | 節點有了，沒有相機 driver |
| `min_turn_radius` | 不適用 | 差速車；R550 才需要 |

R550 預期：原生 m/s、有 encoder 的 odom、STM32 的 IMU、M10P 的 `/scan`；STM32 若只收轉向角，要在 `wro2026_base_r550` 裡把 (v, ω) 換成轉向角（見 [R550_COMPATIBILITY.md](R550_COMPATIBILITY.md)）。
