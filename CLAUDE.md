# CLAUDE.md

Codex 時期的專案規則在 AGENTS.md，模擬（`src/wro2026_sim`）相關工作照常遵守：

@AGENTS.md

---

## 實車開發（2026-10 起）

### 現況
- 實車主機：Jetson Orin Nano，Ubuntu 24.04 + ROS 2 **Jazzy**（L4T R39.2.1，ssh `user2@112ab-jno`）。模擬端也是 Jazzy，但 Jetson 只裝了 `ros-base`，不要假設有 Gazebo 或 GUI 套件。
- 正式的車是 **WHEELTEC R550 mini akm**（阿克曼、STM32 C30D 控制板、encoder、ICM20948 IMU、LSLIDAR N10P、WHEELTEC C70 相機），2026-10-15 到貨。硬體見 `docs/real/R550_HARDWARE.md`。過渡用的 WAVE ROVER 已收起來，相關程式和文件 2026-10-16 刪除。
  - 底盤相關（串口協定、driver）放在獨立 package（`wro2026_base_r550`）。
  - 阿克曼**不能原地旋轉**，底盤以上的程式不得依賴原地旋轉。
- 專題範圍與時程見 `docs/research/PROJECT_PLAN.md`（不參加 WRO，11 月中完成）。
- 硬體型號未確認的項目一律標 `UNKNOWN`，不得猜測補齊。
- 目前狀態只看 `docs/STATUS.md`；文件索引在 `docs/README.md`。2026-10-09 的交接資料已封存在 `docs/archive/handoff_2026-10-09/`（里程碑 H0–H9 的定義在 `START_HERE_NEXT_AI.md`）。
- **實車線不套用 `docs/sim/SPEC.md` 的流程**（三份 SHA 比對、唯讀 archive、Level 0/1/2 都只適用模擬線），只照這份 CLAUDE.md 的安全規則。

### Package 規劃
- `src/wro2026_sim`：模擬，保持原樣，不得部署到 Jetson。
- `src/wro2026_base_r550`：R550 底盤（協定、只讀回授 node、`listen`；指令端之後加）。不得依賴 Gazebo / ros_gz / turtlebot3。
- `src/wro2026_lidar_n10p`：N10P 光達 driver（純 Python、只讀，協定照 LSLIDAR 官方 driver）。
- `src/wro2026_description`：URDF（感測器位置、TF 樹，`base_link` = 後軸中心），不碰硬體。
- Jetson 上用 `~/racecar_ws`（不是 git repo）build / test；操作步驟在 `docs/real/RUNBOOK.md`。Jetson 的 `~/ros2_ws/src/wave_rover_driver` 是舊 driver 的備份，不要修改。

### 安全規則（實車）
- 任何會讓馬達轉動的程式或指令，執行前先跟我確認，並說明：限速、限時、怎麼停。
- 底盤 driver 必須有 `/cmd_vel` 逾時自動停車（預設 0.3 s）和速度上限。
- stop 與 timeout 未在「輪子架空」狀態驗證前，不寫任何讓車在地面自主行駛的 launch。
- 不得把 Gazebo ground-truth odom、模擬 topic 接進實車控制迴路。
- 不得升級 JetPack、改 device tree、bootloader、電源模式。安裝 apt 套件前先列清單給我確認。

### Git
- 每個指令標明在哪台機器跑（WSL / Jetson）。
- 不用 `git reset --hard`、`git push --force`、`git clean`；commit 前給我看 `git diff --stat`。
- 模擬開發照 AGENTS.md 流程：在 ~/ros2_racecar 修改，再 rsync 到 ~/ros2_ws build/launch。Phase 7 campaign 期間的禁止寫入規則已解除。

### 回覆
- 用繁體中文。
