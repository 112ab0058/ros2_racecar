# 專案目前狀態

更新時間：2026-10-17。這是唯一的「目前狀態」文件。專題範圍與時程見 [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md)，每次工作的紀錄見 [research/LOGBOOK.md](research/LOGBOOK.md)。

## 專題

「多感測器融合感知與智慧決策系統」，**不參加 WRO**，**11 月中完成**。正式的車是 **WHEELTEC R550 mini akm + Jetson Orin Nano**。過渡用的 WAVE ROVER 已收起來，它的程式和文件 2026-10-16 從 repo 刪除（git 歷史裡還找得到）。

## 實車（R550）

| 項目 | 狀態 | 文件 |
|---|---|---|
| 硬體盤點 | 大致完成。軸距約 15 cm、輪胎周長約 21.5 cm、光達中心離地約 15 cm；最大轉向角、輪距、外形尺寸 UNKNOWN | [real/R550_HARDWARE.md](real/R550_HARDWARE.md) |
| 廠商資料研讀 | 協定、運動學、開機流程、接線完成；**STM32 原始碼已找到**（無線燒錄工具資料夾裡的 zip，2026-10-19）；Mini 硬體說明還沒下載到 | [research/R550_STUDY_NOTES.md](research/R550_STUDY_NOTES.md) |
| STM32 通訊（只讀） | **2026-10-16 實車通過**：CH9102、115200、約 20 Hz、壞幀 0 | [real/RUNBOOK.md](real/RUNBOOK.md) |
| ROS node（只讀）`/odom`、`/imu/data`、`/battery` | **2026-10-16 實車通過**：20 Hz；里程計比例校正 0.814（架空手轉測的，暫定） | [../src/wro2026_base_r550/README.md](../src/wro2026_base_r550/README.md) |
| 指令端（會讓馬達轉） | **2026-10-19 架空測試全部通過（步驟 1–9）**：Motor OFF 不動、前進／倒車、限速 0.1、轉向正負號（左轉 wz 為正）、不原地轉、停 node 立刻停、限時、低電壓拒絕。0.1 m/s 指令 → 後輪 0.100 m/s；停止發布後 0.3 s 逾時 + 0.8 s 減速。`max_run_time` 新版（從第一個指令算）同晚重測 5.00 s 觸發。**STM32 韌體沒有指令逾時**，上地面前要決定怎麼處理 | [real/RUNBOOK.md](real/RUNBOOK.md) 6b 節 |
| EKF（robot_localization） | 2026-10-20 寫好 `localization.launch.py` + `config/ekf.yaml`（odom vx、vy + IMU vyaw，2D）。**還沒上車跑**：下一步 `orin_check.sh --ekf` | [real/RUNBOOK.md](real/RUNBOOK.md) 6e 節 |
| 鍵盤遙控 | **2026-10-19 架空通過**：放開 → 指令變 0 **0.20 s**、→ 輪子停 **0.96–1.16 s**；`q`/`e`/`z`/`c` 的 wz 正負號全對；`/cmd_vel` 最大間隔 0.052 s，沒斷過。地面行駛等 STM32 逾時定案 | [real/RUNBOOK.md](real/RUNBOOK.md) 6c 節 |
| 光達 N10P | Python driver `wro2026_lidar_n10p`（協定照官方 driver）。**2026-10-17 實車只聽通過**：333 包/秒、每秒 10 圈、壞封包 0。光達 0° 正對車頭、左右方向正確。**`/scan` 10 Hz、480 格實車通過、Foxglove 即時看得到** | [../src/wro2026_lidar_n10p/README.md](../src/wro2026_lidar_n10p/README.md) |
| URDF / TF | **2026-10-17 實車通過**：`base_link → laser_link` = 0.130, 0, 0.115（光達在後軸前 13 cm、離地 15 cm；左右偏移、IMU 位置 UNKNOWN）。`sensors.launch.py` 一次啟動全部只讀 node | [../src/wro2026_description/README.md](../src/wro2026_description/README.md) |
| 相機 C70 | 未開始 | |
| Orin 裝上車、電池供電 | 未解決。廠商作法是電池（10–12.6 V）直接接主控 | [real/R550_HARDWARE.md](real/R550_HARDWARE.md) |
| 套件 robot-localization、slam-toolbox、foxglove-bridge | **2026-10-17 已安裝**（新增 204、移除 0）。Nav2 還沒裝：會換掉 OpenCV，要另外處理 | [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md) 第 8 節 |
| Foxglove 看資料 | 唯讀 bridge，Windows 連 `ws://192.168.0.15:8765` | [real/NETWORK.md](real/NETWORK.md) |

## 下一步

完整規劃（模組總表、順序、要調的參數、實驗）見 [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md)，順序在第 4 節；**每一步的小步驟、誰做、完成標準見 [research/DETAILED_PLAN.md](research/DETAILED_PLAN.md)**。

1. 步驟 7 的校正先用「手推」做（只讀）：~~IMU 靜止~~（10-19 完成）→ `vx_scale`（手推直走）→ IMU 軸向 + `wz_scale`（手推左彎）。工具 `orin_calib.sh`（RUNBOOK 6d）
2. EKF 設定（robot_localization），一樣先用手推驗證
3. Orin 用機殼的調節支架裝上車、電池供電（線材已買；**缺三用電表**，接上 Orin 前要先量電池接頭極性）
4. 場地的牆加高到約 25 cm
5. Orin 上車後：用馬達在地面開，複驗 `vx_scale`、量實際轉彎半徑

## 已知問題

- `flag_stop` 不代表 Motor 開關：原始碼裡是「軟體失能標誌」，自檢錯誤才會變 1（2026-10-19 讀原始碼）
- STM32 韌體沒有指令逾時：node 當掉時車子會照最後的指令跑，只能靠 Motor 開關。**2026-10-19 決定先不改韌體**，上地面時靠操作防護（限速 ≤ 0.15 m/s、有牆的場地、手在 Motor 開關旁、USB 線固定）；Orin 上車前不在地面用馬達開車（拖線會扯掉 USB）
- Jetson 沒有 Python 的 OpenCV（`python3-opencv`）和 cv_bridge；相機程式開始前要決定怎麼裝
- WSL 在 NAT 模式下看不到 Orin 的 topic，用 Foxglove 看（[real/NETWORK.md](real/NETWORK.md)）
- Orin 的 `~/racecar_ws/src/` 裡還有舊 package（WAVE ROVER 時期），不影響 `--packages-select` 的 build，要清的話 Ray 自己在 Orin 刪；`~/ros2_ws` 是舊 driver 的備份，不要動

## 模擬線（`src/wro2026_sim`）：已結束，只當參考

Gazebo 模擬在 2026-07 完成（Phase 6J 驗收），Phase 7 的 odom 校正在模擬模型中證明不可行。程式不移植到實車、不修改。規則見 [sim/SPEC.md](sim/SPEC.md)，歷史紀錄在 [archive/](archive/)。
