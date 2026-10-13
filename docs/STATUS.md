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

1. ~~人工目視確認（H2，HTTP）~~：2026-10-13 完成，方向正確。
2. 2026-10-14 接 UART（Orin pin 8/10/6 ↔ 車頂 RX/TX/GND，`/dev/ttyTHS1`），照 [RUNBOOK 第 4 節](real/RUNBOOK.md)用新 driver 重跑 H2 的 serial 版本。
3. 量測輪距、輪徑、`pwm_to_mps`、IMU 方向，補上 UNKNOWN（[RUNBOOK 第 5 節](real/RUNBOOK.md)）。
4. 決定相機型號並裝相機 driver，才能在實車上跑 line detector。
5. 分支 `control/pose-free-race-controller`（2026-10-13，WIP）有不依賴座標的比賽控制器 `wro2026_control`。合併前要先解決：它訂閱 `/imu`，底盤 driver 發佈 `/imu/data`；它輸出 m/s 的 `/cmd_vel`，底盤 driver 目前把 `/cmd_vel` 當 PWM 比例（見已知問題）；它需要 `/scan`（LiDAR）和行駛距離（需要 `pwm_to_mps`）。

## 已知問題

- `src/wro2026_sim/config/phase7/` 裡有 Python 腳本（`localization_probe.py`、`mapping_motion_supervisor.py`、`mapping_traversal.py`），放在 config 資料夾不合理，但搬移有風險，暫不動。
- `src/wro2026_sim/config/phase7/localization_validation_route.yaml` 的 `source_evidence` 只是紀錄欄位，不影響執行；它指向的證據檔案（`~/ros2_evidence/...`）只存在 Ray 的 WSL 電腦上，不在 repo 裡。歷史 Phase 的 evidence 也都在那台電腦。
- 模擬 package 沒有任何自動測試（實車的兩個 package 有）。
- `/line_detection` 的 Float32MultiArray 格式只靠位置表示意義，很脆弱，之後應改成自訂 message。
- 底盤 driver 把 `/cmd_vel` 的 `linear.x`、`angular.z` 直接當 PWM 比例（沿用舊 driver），但 `/odom` 的速度是 m/s；上層目前必須知道這個底盤的單位，違反「上層只靠 `/cmd_vel` 溝通、不依賴底盤」的原則。`pwm_to_mps` 量好後應改成 m/s → PWM 換算。
- 新舊 driver 的互斥檢查只在新 driver 和 `test_motion` 裡；先開新 driver、再開舊 driver 時，舊的不會擋（舊 driver 不能改），要靠人確認。
- Jetson 沒有 OpenCV / cv_bridge：line detector 只支援 rgb8 / bgr8 / rgba8 / bgra8 影像。
- Ray 的 WSL `~/ros2_racecar`（2026-10-13 檢查）：未追蹤的 HARDWARE_INVENTORY.md 已搬進 repo 成為 [real/FINAL_CAR_DRAFT.md](real/FINAL_CAR_DRAFT.md)；`backup/local-main-20261009` 的 2 個 commit 經 `git cherry` 確認在 main 上都有相同修改；stash 是 2026-07 的舊版控制器修改（main 已經更新過）。三者都已在 Ray 的 Windows 下載資料夾留有 patch 備份。
- 封存文件的內文沒有改，裡面提到的 `docs/` 路徑（例如舊的 CURRENT_STATE 位置）指的是 2026-10-13 搬移前的位置。
- `archive/handoff_2026-10-09/SHA256SUMS.txt` 是封存前內容的雜湊；每個 .md 去掉第一行封存標記與後面的空行（`tail -n +3`）後仍然相符。
