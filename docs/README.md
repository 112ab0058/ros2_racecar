# 文件索引

先讀 [STATUS.md](STATUS.md)。

## 研究（`research/`）：專題報告的素材

| 文件 | 用途 |
|---|---|
| [research/PROJECT_PLAN.md](research/PROJECT_PLAN.md) | **專題範圍、系統架構、已決定 / 待決定的事、4 週時程、風險** |
| [research/LOGBOOK.md](research/LOGBOOK.md) | 研究日誌：每次工作的目的、做法、數據、問題、學到的東西 |
| [research/R550_STUDY_NOTES.md](research/R550_STUDY_NOTES.md) | 廠商資料研讀筆記：C30D、開機流程、阿克曼運動學、馬達 PI、電源、**串口協定**、Orin 怎麼接車子 |
| [research/LEARNING_PATH.md](research/LEARNING_PATH.md) | 各階段要懂的觀念和小練習（時程以 PROJECT_PLAN 為準） |

## 實車（`real/`）

| 文件 | 用途 |
|---|---|
| [real/R550_HARDWARE.md](real/R550_HARDWARE.md) | R550 mini akm 實物盤點：C30D、N10P 光達、C70 相機、尺寸、電源 |
| [real/RUNBOOK.md](real/RUNBOOK.md) | 操作手冊：部署、build、開機順序、只聽 STM32、啟動 node、充電、疑難排解 |
| [real/NETWORK.md](real/NETWORK.md) | 網路：IP / MAC、路由器綁定、NetBird、Foxglove、WSL 看不到 Orin 的原因、`ROS_DOMAIN_ID` |
| [real/ORIN_BACKUP.md](real/ORIN_BACKUP.md) | Orin 狀態備份與 SD 卡映像的做法、還原步驟 |
| [../src/wro2026_base_r550/README.md](../src/wro2026_base_r550/README.md) | R550 底盤 package：協定、只讀 node、`listen` 工具 |
| [../src/wro2026_lidar_n10p/README.md](../src/wro2026_lidar_n10p/README.md) | N10P 光達 driver（純 Python、只讀）：協定、`/scan` node、`listen` 工具 |

## 模擬（`sim/`，已結束，只當參考）

| 文件 | 用途 |
|---|---|
| [sim/FIELD_PLAN_160CM.md](sim/FIELD_PLAN_160CM.md) | 160 × 160 cm 場地的尺寸與材料（**實體場地也用這份**） |
| [sim/SPEC.md](sim/SPEC.md) | 模擬線的 Phase 規格；實車不套用 |
| [sim/GAZEBO_PERFORMANCE.md](sim/GAZEBO_PERFORMANCE.md) | WSL 上 Gazebo 的效能設定 |
| [sim/ROS2_WSL_NETWORK.md](sim/ROS2_WSL_NETWORK.md) | WSL 的 ROS 2 網路設定 |
| [sim/WORKSPACE_MAP.md](sim/WORKSPACE_MAP.md) | WSL 工作區與 repo 的對照 |

## 封存（`archive/`，內文不再更新）

| 文件 | 用途 |
|---|---|
| [archive/sim_phases/](archive/sim_phases/CURRENT_STATE.md) | 模擬 Phase 2–7 的狀態與證據摘要 |
| [archive/handoff_2026-10-09/](archive/handoff_2026-10-09/README.md) | 2026-10-09 從模擬轉實車的交接包 |
