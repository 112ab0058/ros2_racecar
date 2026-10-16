# 專案目前狀態

更新時間：2026-10-13。這是唯一的「目前狀態」文件；舊的狀態文件已封存在 [archive/sim_phases/](archive/sim_phases/)。
本頁只整理現有文件的內容，查不到的寫 `UNKNOWN`。

## 模擬線（`src/wro2026_sim`）：已結束，只當參考

| 項目 | 狀態 | 來源 |
|---|---|---|
| Phase 6J 最終驗收 | **J1 FINAL_ACCEPTANCE_PASS；FINAL ACCEPTED**（2026-07-22） | [CURRENT_STATE](archive/sim_phases/CURRENT_STATE.md) |
| 已驗收 controller | `src/wro2026_sim/scripts/line_follower.py`，SHA-256 `de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2` | 同上 |
| Phase 7B/7C/7D | 場地基線、TF/sensor/odom 基線、SLAM mapping 通過（7C 曾發現 IMU 0 messages、depth 缺失、odom covariance 全零） | [PROJECT_HANDOFF](archive/handoff_2026-10-09/PROJECT_HANDOFF.md) |
| Phase 7E-AE | 輪子幾何校正：`7EAE2 WHEEL_GEOMETRY_CALIBRATION_NOT_IDENTIFIABLE_OR_NOT_GENERALIZABLE`；campaign 結論 `PROVEN_IRREDUCIBLE_ARCHITECTURE_BLOCKER` | 同上、[KNOWN_PATHS](archive/handoff_2026-10-09/KNOWN_PATHS_AND_EVIDENCE.md) |
| Phase 7G | residual odom：`PROVEN_RESIDUAL_ODOM_ARCHITECTURE_BLOCKER`（`STOP_RUNTIME_BUDGET_EXHAUSTED`）；runtime slots 已用完，不得重跑；7F 未進入 | 同上 |

**Phase 7 的意義**：只證明在凍結的模擬模型、特定 excitation、固定 gate 與有限 runtime 預算下，這兩條 odom 校正路線沒有產生可接受的候選；**不代表**實車、Nav2 或所有 odometry 方法不可行。實車要從真實硬體重新量測開始。

`line_follower.py`（4315 行、279 個模組層級常數，依模擬場地的絕對座標做決策）只能當參考，不移植到實車、不修改。
模擬線的工作流程（三份 SHA 比對、唯讀 archive、Level 0/1/2）見 [sim/SPEC.md](sim/SPEC.md)，**只適用模擬線**。

## 實車線：Jetson Orin Nano + Waveshare WAVE ROVER（過渡測試車）

硬體細節見 [real/REAL_CAR_HARDWARE.md](real/REAL_CAR_HARDWARE.md)，操作步驟見 [real/RUNBOOK.md](real/RUNBOOK.md)。里程碑定義來自 [START_HERE_NEXT_AI](archive/handoff_2026-10-09/START_HERE_NEXT_AI.md)。
程式：`src/wro2026_base_waverover`（底盤 driver，由 Jetson 的 `wave_rover_driver` 搬進來）、`src/wro2026_real`（bringup、URDF、參數化 line detector）。兩個 package 都有不需硬體的 pytest（Jetson `colcon test`：0 失敗，數字見 [2026-10-13 報告](reports/2026-10-13_cleanup_and_wave_rover_base.md)）。

| 里程碑 | 狀態 | 說明 |
|---|---|---|
| H0 硬體盤點 | DONE（過渡車） | WAVE ROVER + Orin 已盤點；UNKNOWN 7 項（輪距/輪徑/速度比例、車上供電、相機、LiDAR、比賽車、急停、IMU 軸向）。比賽車另有草稿 [real/FINAL_CAR_DRAFT.md](real/FINAL_CAR_DRAFT.md)（Ackermann、STM32、Orbbec 深度相機、LiDAR、encoder；型號大多 UNKNOWN） |
| H1 Jetson / OS / ROS 2 | DONE | Ubuntu 24.04.4、L4T R39.2.1、ROS 2 Jazzy `ros-base` + `ros-dev-tools`；沒有 Gazebo / GUI |
| H2 架空低速與停止 | PARTIAL | 舊 driver 2026-10-13 HTTP、max_speed 0.05、架空：啟動只送停止、前後左右、`/cmd_vel` 中斷 0.306 s 停車、關閉送停止全部 PASS。新 driver 加了回授遺失停車與低電壓拒絕（pytest 驗證），HTTP 只讀實機驗證通過。2026-10-13 11:14 新 driver HTTP、0.05、架空跑 `test_motion`：B1–B4 全部 PASS，**使用者目視確認四個動作方向正確、停得住**。剩下：UART 版本（預計 2026-10-14） |
| H3 encoder odometry | BLOCKED（這台車） | WAVE ROVER 沒有 encoder。過渡用的 dead reckoning（送出的輪指令 × `pwm_to_mps` + IMU gz）已實作；`pwm_to_mps` UNKNOWN（預設 0，只轉向不位移）；低品質，不可作為 Nav2 主要 odom |
| H4 感測器 driver / TF | PARTIAL | `/imu/data`、`/battery`、`/diagnostics`、TF `odom → base_link → imu_link / camera_link` 實機確認；IMU gz 靜止雜訊大（標準差 5.1 deg/s）；相機型號 UNKNOWN、沒有相機 driver；LiDAR 未安裝 |
| H5 URDF 與實際尺寸 | PARTIAL | 最小 URDF（`base_link`、`imu_link`、`camera_link`）已建立，尺寸全是暫定值；輪距、輪徑、感測器位置 UNKNOWN |
| H6 localization | 未開始 | |
| H7 mapping / Nav2 | 未開始 | |
| H8 WRO 行為 | 未開始 | |
| H9 冷啟動 / 故障測試 / 驗收 | 未開始 | |

## 下一步

**2026-10-16 專題範圍確定**：不參加 WRO，11 月中完成；用 slam_toolbox + Nav2 做建圖導航，不用 YOLO，再加 AI 任務層。範圍、架構、4 週時程見 [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md)，以它為準。同日新增 `wro2026_base_r550` 的**只讀**回授 node 和 `listen` 工具（**2026-10-16 實車只聽測試通過**：20 Hz、壞幀 0），第一次接車步驟在 [package README](../src/wro2026_base_r550/README.md)；下面 A、B、C 是之前的紀錄。

**2026-10-15 R550 到貨**：型號是 **R550 mini akm**，控制板 C30D（STM32），光達是 **LSLIDAR N10P（不是 M10P）**，相機是 **WHEELTEC C70**（RGB、720P、UVC）。硬體盤點見 [real/R550_HARDWARE.md](real/R550_HARDWARE.md)；接下來照 [research/LEARNING_PATH.md](research/LEARNING_PATH.md) 的 5 個階段進行，過程記在 [research/LOGBOOK.md](research/LOGBOOK.md)。A、B 是到貨前的紀錄。

### A. 現在：WAVE ROVER
1. ~~人工目視確認（H2，HTTP）~~：2026-10-13 完成，方向正確。
2. 2026-10-14 接 UART（Orin pin 8/10/6 ↔ 車頂 RX/TX/GND，`/dev/ttyTHS1`），照 [RUNBOOK 第 4 節](real/RUNBOOK.md)跑 serial 版的 H2。
3. 決定相機：WRO 的線和柱子都要靠顏色辨識，光達做不到。先用任何 USB 攝影機接 Orin，在 WAVE ROVER 上測 line detector。
4. 暫停：`pwm_to_mps` 地面校正、IMU 濾波（R550 有 encoder，到時候直接做真正的里程計）。

### B. 到貨前在 Orin 上準備（不需要 R550）
2026-10-13～14 的結果見 [R550 準備報告](reports/2026-10-13_r550_prep.md)。
1. **B1 Orin 備份：DONE（狀態備份）**。套件清單、apt 來源、原始碼已備份到 Orin 與 Ray 的 Windows（[real/ORIN_BACKUP.md](real/ORIN_BACKUP.md)）；完整 SD 卡映像要 Ray 關機拔卡自己做（步驟在同一份文件）。
2. **B2 ROS 2 套件：BLOCKED**。模擬安裝顯示 `ros-jazzy-nav2-bringup` 會把 Ubuntu 的 OpenCV 4.6 開發套件換成 NVIDIA 的 4.8（移除 10 個套件），依規則沒有安裝。`rosbag2-storage-mcap`、`tf2-tools` 本來就有；其餘 3 個（robot-localization、slam-toolbox、foxglove-bridge）一起模擬：移除 0 個、升級的 7 個都不是 nvidia / cuda / tensorrt / libnv / linux- 開頭（也沒有 NVIDIA 來源或 OpenCV），兩項檢查都通過；要不要裝由 Ray 決定。
3. **B3 R550 / M10P 相容性：PARTIAL**。調查完成，但 M10P 官方 driver 在 Jetson 上 build 失敗（缺 Boost，依 CMakeLists 推測還缺 PCL / pcl_conversions / pcap，都沒裝）。沒找到 WHEELTEC 公開的 R550 官方程式；STM32 協定來自兩份同源（都源自 WHEELTEC 原廠 driver）的第三方程式，彼此一致，但都不是 R550 / C30D 的資料，寫成 `src/wro2026_base_r550`。**2026-10-16 找到 WHEELTEC 官方協定文件，編碼 / 解碼和文件裡的範例封包全部一致**（還沒在實車錄封包驗證）。M10P 官方 driver 的 README 寫支援 Jazzy。見 [real/R550_COMPATIBILITY.md](real/R550_COMPATIBILITY.md)。
4. **B4 網路：PARTIAL**。IP / MAC、DHCP 綁定步驟、NetBird 現況、WSL 看不到 Orin 的原因寫在 [real/NETWORK.md](real/NETWORK.md)；bringup 的唯讀 Foxglove（`foxglove:=true`）已加，但 `foxglove_bridge` 還沒裝，沒能實際連線。路由器綁定、NetBird 邀請要 Ray / 組員處理。
5. **B5 控制器整合：PARTIAL**。`wro2026_control` 已 merge，topic 名稱改成參數（IMU 預設 `/imu/data`），離線測試、sim2d、matrix 的結果修改前後完全相同（這些只測 `core.py`；改過的 `race_node.py` / `race_monitor.py` 另用靜態測試檢查 topic 接線）；WAVE ROVER driver 加了 `cmd_vel_units`（`pwm` / `mps`）；介面合約在 [real/INTERFACES.md](real/INTERFACES.md)。Jetson 沒有 Python 的 OpenCV（`python3-opencv`，`import cv2` 失敗；C++ 的 OpenCV 4.6 有裝），所以控制器在 Jetson 上不能跑、`test_core.py` 收集失敗；WAVE ROVER 沒有位移 odom 和光達，不能跑控制器。**控制器的 `start` 參數預設是 true**，之後在實車上一律要加 `start:=false`。

### C. R550 到貨後（詳細步驟見 [research/LEARNING_PATH.md](research/LEARNING_PATH.md)）
1. H0：拍照、記錄版本、電池與給 Orin 的供電方式、STM32 韌體與通訊協定。2026-10-15 依照片完成初步盤點（[real/R550_HARDWARE.md](real/R550_HARDWARE.md)）；產品資料包已下載（2026-10-16，STM32 原始碼、Mini 硬體說明、ROS 2 教學還缺），研讀筆記見 [research/R550_STUDY_NOTES.md](research/R550_STUDY_NOTES.md)。**下一步**：Motor 開關放 OFF、開機拍 OLED，再只聽 STM32 封包，和官方協定比對。
2. 底盤 package `wro2026_base_r550`：沿用 WAVE ROVER driver 的安全機制（`/cmd_vel` 逾時、限速、回授遺失、低電壓），架空測試 stop / timeout。
3. 阿克曼：官方文件寫 STM32 收 (v, ω)，自己換算轉角（2026-10-16）；還要架空實測方向。量軸距、最大轉向角，寫進 URDF（廠商程式註解：mini 阿克曼軸距 0.144 m、輪距 0.162 m，待量測）。
4. 光達 driver（**N10P**，B3 查的是 M10P，要重查；官方 `Lslidar_ROS2_driver` 有列 N10 系列）+ 相機 driver（C70 是 UVC）+ TF；encoder odom + IMU 用 `robot_localization` 融合。
5. 接上 `wro2026_control`（pose-free 控制器），先低速、再地面。
6. 之後才是 SLAM / Nav2（H6–H7）。

## 已知問題

- `src/wro2026_sim/config/phase7/` 裡有 Python 腳本（`localization_probe.py`、`mapping_motion_supervisor.py`、`mapping_traversal.py`），放在 config 資料夾不合理，但搬移有風險，暫不動。
- `src/wro2026_sim/config/phase7/localization_validation_route.yaml` 的 `source_evidence` 只是紀錄欄位，不影響執行；它指向的證據檔案（`~/ros2_evidence/...`）只存在 Ray 的 WSL 電腦上，不在 repo 裡。歷史 Phase 的 evidence 也都在那台電腦。
- 模擬 package 沒有任何自動測試（實車的兩個 package 有）。
- `/line_detection` 的 Float32MultiArray 格式只靠位置表示意義，很脆弱，之後應改成自訂 message。
- 底盤 driver 預設把 `/cmd_vel` 的 `linear.x`、`angular.z` 直接當 PWM 比例（`cmd_vel_units: pwm`，沿用舊 driver），但 `/odom` 的速度是 m/s。2026-10-14 已加 `cmd_vel_units: mps`，要等 `pwm_to_mps` 實測後才能用（是 0 時 driver 拒絕啟動）。
- 新舊 driver 的互斥檢查只在新 driver 和 `test_motion` 裡；先開新 driver、再開舊 driver 時，舊的不會擋（舊 driver 不能改），要靠人確認。
- Jetson 沒有 Python 的 OpenCV（`python3-opencv`）和 cv_bridge（C++ 的 OpenCV 4.6 開發套件有裝）：line detector 只支援 rgb8 / bgr8 / rgba8 / bgra8 影像；`wro2026_control` 不能在 Jetson 上跑。
- Ray 的 WSL `~/ros2_racecar`（2026-10-13 檢查）：未追蹤的 HARDWARE_INVENTORY.md 已搬進 repo 成為 [real/FINAL_CAR_DRAFT.md](real/FINAL_CAR_DRAFT.md)；`backup/local-main-20261009` 的 2 個 commit 經 `git cherry` 確認在 main 上都有相同修改；stash 是 2026-07 的舊版控制器修改（main 已經更新過）。三者都已在 Ray 的 Windows 下載資料夾留有 patch 備份。
- 封存文件的內文沒有改，裡面提到的 `docs/` 路徑（例如舊的 CURRENT_STATE 位置）指的是 2026-10-13 搬移前的位置。
- `archive/handoff_2026-10-09/SHA256SUMS.txt` 是封存前內容的雜湊；每個 .md 去掉第一行封存標記與後面的空行（`tail -n +3`）後仍然相符。
