# CLAUDE.md

Codex 時期的專案規則在 AGENTS.md，模擬（`src/wro2026_sim`）相關工作照常遵守：

@AGENTS.md

---

## 實車開發（2026-10 起）

### 現況
- 實車主機：Jetson Orin Nano，Ubuntu 24.04 + ROS 2 **Jazzy**（L4T R39.2.1，ssh `user2@112ab-jno`）。模擬端也是 Jazzy，但 Jetson 只裝了 `ros-base`，不要假設有 Gazebo 或 GUI 套件。
- 目前是**過渡測試車：Waveshare WAVE ROVER（ESP32、差速、沒有 encoder、尚未裝光達）**。硬體細節與已驗證項目見 `docs/REAL_CAR_HARDWARE.md`。最後比賽車是轉向輪（Ackermann 類），所以：
  - 底盤相關（馬達、encoder、運動學）放在獨立 package，之後換車只換這層。
  - 底盤以上（相機、line detection、決策）只透過 `/cmd_vel`、`/odom`、TF、感測器 topic 溝通，不得依賴差速專屬行為（例如原地旋轉）。
- 硬體型號未確認的項目一律標 `UNKNOWN`，不得猜測補齊。
- 交接資料在 `docs/handoff_2026-10-09/`，先讀 `START_HERE_NEXT_AI.md`。

### Package 規劃
- `src/wro2026_sim`：模擬，保持原樣，不得部署到 Jetson。
- `src/wro2026_real`（待建立）：實車 bringup、底盤 driver、相機、參數化的 line detector。不得依賴 Gazebo / ros_gz / turtlebot3。
- WAVE ROVER 底盤 driver 目前在 Jetson 的 `~/ros2_ws/src/wave_rover_driver`（本機 git，未進這個 repo），之後要搬進 `src/` 成為獨立的底盤 package。

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
