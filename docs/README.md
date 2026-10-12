# 文件索引

先讀 [STATUS.md](STATUS.md)。

## 主線

| 文件 | 用途 |
|---|---|
| [STATUS.md](STATUS.md) | 唯一的「目前狀態」：模擬線結論、實車里程碑 H0–H9、下一步、已知問題 |
| [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) | 系統架構、package 分工與資料流（模擬與實車） |

## 實車（`real/`）

| 文件 | 用途 |
|---|---|
| [real/REAL_CAR_HARDWARE.md](real/REAL_CAR_HARDWARE.md) | Jetson 與 WAVE ROVER 的硬體盤點、允許的 ESP32 指令、已驗證項目、UNKNOWN 清單 |
| [real/RUNBOOK.md](real/RUNBOOK.md) | 實車操作：部署、啟動 / 停止、架空目視測試、UART 接線後的步驟、量測輪距與 `pwm_to_mps` |
| [../src/wro2026_base_waverover/README.md](../src/wro2026_base_waverover/README.md) | 底盤 driver：結構、topic、安全行為、odometry 的限制 |
| [../src/wro2026_real/README.md](../src/wro2026_real/README.md) | bringup、URDF、line detector 與 `/line_detection` 格式 |

## 模擬（`sim/`）

| 文件 | 用途 |
|---|---|
| [sim/SPEC.md](sim/SPEC.md) | 模擬線的 Phase 執行規格（SHA 比對、archive、Level 0/1/2）；實車線不套用 |
| [sim/FIELD_PLAN_160CM.md](sim/FIELD_PLAN_160CM.md) | 160 × 160 cm 場地尺寸與材料規劃 |
| [sim/GAZEBO_PERFORMANCE.md](sim/GAZEBO_PERFORMANCE.md) | WSL 上 Gazebo 的效能設定清單 |
| [sim/ROS2_WSL_NETWORK.md](sim/ROS2_WSL_NETWORK.md) | WSL 的 ROS 2 網路設定（描述 Ray 的 WSL 環境） |
| [sim/WORKSPACE_MAP.md](sim/WORKSPACE_MAP.md) | WSL 工作區與 GitHub repo 的對照（描述 Ray 的 WSL 環境） |

## 封存（`archive/`，內文不再更新）

| 文件 | 用途 |
|---|---|
| [archive/sim_phases/CURRENT_STATE.md](archive/sim_phases/CURRENT_STATE.md) | 模擬 Phase 2–6J 的權威狀態（停在 2026-07-22） |
| [archive/sim_phases/TEST_EVIDENCE.md](archive/sim_phases/TEST_EVIDENCE.md) | 模擬各 Phase 的測試證據摘要（停在 2026-07-22） |
| [archive/sim_phases/NEXT_PROMPT.md](archive/sim_phases/NEXT_PROMPT.md) | 2026-07-06 的下一輪 Codex prompt（已過期） |
| [archive/sim_phases/AI_SYNC.md](archive/sim_phases/AI_SYNC.md) | 2026-07-06 的 AI 同步紀錄（已過期） |
| [archive/sim_phases/NEXT_PROMPT_phase7g.md](archive/sim_phases/NEXT_PROMPT_phase7g.md) | Phase 7G residual odom campaign 的完整 prompt |
| [archive/handoff_2026-10-09/](archive/handoff_2026-10-09/README.md) | 2026-10-09 從模擬轉實車的交接包（H0–H9 里程碑定義、Phase 7 結論、證據路徑） |
