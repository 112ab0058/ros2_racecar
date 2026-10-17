# 專案目前狀態

更新時間：2026-10-17。這是唯一的「目前狀態」文件。專題範圍與時程見 [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md)，每次工作的紀錄見 [research/LOGBOOK.md](research/LOGBOOK.md)。

## 專題

「多感測器融合感知與智慧決策系統」，**不參加 WRO**，**11 月中完成**。正式的車是 **WHEELTEC R550 mini akm + Jetson Orin Nano**。過渡用的 WAVE ROVER 已收起來，它的程式和文件 2026-10-16 從 repo 刪除（git 歷史裡還找得到）。

## 實車（R550）

| 項目 | 狀態 | 文件 |
|---|---|---|
| 硬體盤點 | 大致完成。軸距約 15 cm、輪胎周長約 21.5 cm、光達中心離地約 15 cm；最大轉向角、輪距、外形尺寸 UNKNOWN | [real/R550_HARDWARE.md](real/R550_HARDWARE.md) |
| 廠商資料研讀 | 協定、運動學、開機流程、接線完成；STM32 原始碼、Mini 硬體說明還沒下載到 | [research/R550_STUDY_NOTES.md](research/R550_STUDY_NOTES.md) |
| STM32 通訊（只讀） | **2026-10-16 實車通過**：CH9102、115200、約 20 Hz、壞幀 0 | [real/RUNBOOK.md](real/RUNBOOK.md) |
| ROS node（只讀）`/odom`、`/imu/data`、`/battery` | **2026-10-16 實車通過**：20 Hz；里程計比例校正 0.814（架空手轉測的，暫定） | [../src/wro2026_base_r550/README.md](../src/wro2026_base_r550/README.md) |
| 指令端（會讓馬達轉） | **2026-10-17 程式寫好、單元測試通過，還沒架空驗證**：`base_node`（`enable_commands` 預設 false）、逾時 0.3 s、限速、回授遺失、低電壓、限時、不原地轉；架空測試腳本 `orin_air_test.sh` | [real/RUNBOOK.md](real/RUNBOOK.md) 6b 節 |
| 光達 N10P | Python driver `wro2026_lidar_n10p`（協定照官方 driver）。**2026-10-17 實車只聽通過**：333 包/秒、每秒 10 圈、壞封包 0。光達 0° 正對車頭、左右方向正確。**`/scan` 10 Hz、480 格實車通過、Foxglove 即時看得到** | [../src/wro2026_lidar_n10p/README.md](../src/wro2026_lidar_n10p/README.md) |
| URDF / TF | **2026-10-17 實車通過**：`base_link → laser_link` = 0.130, 0, 0.115（光達在後軸前 13 cm、離地 15 cm；左右偏移、IMU 位置 UNKNOWN）。`sensors.launch.py` 一次啟動全部只讀 node | [../src/wro2026_description/README.md](../src/wro2026_description/README.md) |
| 相機 C70 | 未開始 | |
| Orin 裝上車、電池供電 | 未解決。廠商作法是電池（10–12.6 V）直接接主控 | [real/R550_HARDWARE.md](real/R550_HARDWARE.md) |
| 套件 robot-localization、slam-toolbox、foxglove-bridge | **2026-10-17 已安裝**（新增 204、移除 0）。Nav2 還沒裝：會換掉 OpenCV，要另外處理 | [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md) 第 8 節 |
| Foxglove 看資料 | 唯讀 bridge，Windows 連 `ws://192.168.0.15:8765` | [real/NETWORK.md](real/NETWORK.md) |

## 下一步

完整規劃（模組總表、順序、要調的參數、實驗）見 [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md)，順序在第 4 節。

1. 指令端架空測試（`orin_air_test.sh`，先跟 Claude 確認）
2. Orin 用機殼的調節支架裝上車、電池供電（等延長線、需要電池接頭的照片）
3. 場地的牆加高到約 25 cm
4. 地面校正里程計、量最大轉向角

## 已知問題

- `flag_stop` 不代表 Motor 開關（開關 OFF 時也是 0），意義要看 STM32 原始碼
- Jetson 沒有 Python 的 OpenCV（`python3-opencv`）和 cv_bridge；相機程式開始前要決定怎麼裝
- WSL 在 NAT 模式下看不到 Orin 的 topic，用 Foxglove 看（[real/NETWORK.md](real/NETWORK.md)）
- Orin 的 `~/racecar_ws/src/` 裡還有舊 package（WAVE ROVER 時期），不影響 `--packages-select` 的 build，要清的話 Ray 自己在 Orin 刪；`~/ros2_ws` 是舊 driver 的備份，不要動

## 模擬線（`src/wro2026_sim`）：已結束，只當參考

Gazebo 模擬在 2026-07 完成（Phase 6J 驗收），Phase 7 的 odom 校正在模擬模型中證明不可行。程式不移植到實車、不修改。規則見 [sim/SPEC.md](sim/SPEC.md)，歷史紀錄在 [archive/](archive/)。
