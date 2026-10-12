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

硬體細節見 [real/REAL_CAR_HARDWARE.md](real/REAL_CAR_HARDWARE.md)。里程碑定義來自 [START_HERE_NEXT_AI](archive/handoff_2026-10-09/START_HERE_NEXT_AI.md)。

| 里程碑 | 狀態 | 說明 |
|---|---|---|
| H0 硬體盤點 | DONE（過渡車） | WAVE ROVER + Orin 已盤點；UNKNOWN 6 項（輪距/輪徑/速度比例、車上供電、相機、LiDAR、比賽車、急停） |
| H1 Jetson / OS / ROS 2 | DONE | Ubuntu 24.04.4、L4T R39.2.1、ROS 2 Jazzy `ros-base` + `ros-dev-tools`；沒有 Gazebo / GUI |
| H2 架空低速與停止 | PARTIAL | 2026-10-13 HTTP、max_speed 0.05、架空：啟動只送停止、前後左右、`/cmd_vel` 中斷 0.306 s 停車、關閉送停止全部 PASS。**輪子是否真的轉動待人工目視**；UART 接線預計 2026-10-14 |
| H3 encoder odometry | BLOCKED（這台車） | WAVE ROVER 沒有 encoder；過渡期只能用 IMU yaw + 指令速度做 dead reckoning，不可作為 Nav2 主要 odom |
| H4 感測器 driver / TF | PARTIAL | IMU 可由 T:126 讀到；相機型號 UNKNOWN；LiDAR 未安裝 |
| H5 URDF 與實際尺寸 | 未開始 | 輪距、輪徑、感測器位置 UNKNOWN |
| H6 localization | 未開始 | |
| H7 mapping / Nav2 | 未開始 | |
| H8 WRO 行為 | 未開始 | |
| H9 冷啟動 / 故障測試 / 驗收 | 未開始 | |

## 下一步

1. 人工目視確認 0.05 PWM 時輪子有轉、方向正確、停得住（H2）。
2. 2026-10-14 接 UART（Orin pin 8/10/6 ↔ 車頂 RX/TX/GND，`/dev/ttyTHS1`），重跑 H2 的 serial 版本。
3. 量測輪距、輪徑、PWM → m/s 對應，補上 UNKNOWN。

## 已知問題

- `src/wro2026_sim/config/phase7/` 裡有 Python 腳本（`localization_probe.py`、`mapping_motion_supervisor.py`、`mapping_traversal.py`），放在 config 資料夾不合理，但搬移有風險，暫不動。
- `src/wro2026_sim/config/phase7/localization_validation_route.yaml` 的 `source_evidence` 只是紀錄欄位，不影響執行；它指向的證據檔案（`~/ros2_evidence/...`）只存在 Ray 的 WSL 電腦上，不在 repo 裡。歷史 Phase 的 evidence 也都在那台電腦。
- 模擬 package 沒有任何自動測試。
- 封存文件的內文沒有改，裡面提到的 `docs/` 路徑（例如舊的 CURRENT_STATE 位置）指的是 2026-10-13 搬移前的位置。
