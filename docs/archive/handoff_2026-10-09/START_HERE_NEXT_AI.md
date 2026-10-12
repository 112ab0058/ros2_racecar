> 已封存（2026-10-13）。目前狀態見 docs/STATUS.md。

# START HERE — 給下一個 AI 的指令

使用者現在手邊有一台實體車與 NVIDIA Jetson Orin，目標是從模擬研究轉入正式實車開發。不要再把主線放在舊模擬器的 wheel calibration/residual odom 負結果；應先辨識硬體，建立安全、可重現的低階控制，再逐層加入 ROS 2、感測器、定位與 Nav2。

## 第一件事：確認硬體，不能猜
請取得或請使用者拍照：
- Jetson 型號：Orin Nano/NX/AGX Orin、RAM、carrier board、JetPack/L4T。
- 車體運動學：differential、Ackermann 或其他。
- 馬達、encoder、driver、MCU 型號與說明書。
- LiDAR、相機、IMU 型號、介面、供電。
- 電池、DC-DC、電源分配、保險絲、急停/快速斷電方式。
- 車體尺寸、輪徑、輪距/軸距、encoder CPR/gear ratio、最大轉向角。
不知道就標 UNKNOWN，不得用猜測補齊。

## 第二件事：只讀盤點現有程式
在 `/home/ray/ros2_racecar` 先執行：
```bash
pwd
git status --short --branch
git diff --check
git log --oneline -20
```
再讀：
- `docs/SPEC.md`
- `docs/CURRENT_STATE.md`
- `docs/TEST_EVIDENCE.md`
- `docs/NEXT_PROMPT.md`

盤點 `src/`, launch/config/URDF/maps，核對實際存在的檔案、HEAD、production source/workspace/installed SHA。只讀檢查，不先 build、launch、刪除、reset、checkout、stash、clean 或覆寫 docs。`/home/ray/ros2_ws` 禁止寫入。

## 第三件事：先交付硬體盤點
先整理 `HARDWARE_INVENTORY.md` 草稿（未經確認先不要覆寫既有 docs），包括硬體型號、電源樹、介面/電平、資料流、安全 stop、未知項目與最低風險測試。

## 實車開發里程碑
H0 硬體盤點（不動馬達）→ H1 Jetson/OS/ROS 2 baseline → H2 車輪架空的 MCU/driver 低速測試與 stop 驗證 → H3 encoder 方向/尺度與 odometry → H4 LiDAR/camera/IMU drivers、時間戳與 TF → H5 URDF/Xacro 和真實尺寸 → H6 localization → H7 mapping/Nav2 → H8 WRO behaviors → H9 多次冷啟動、故障測試與 acceptance。

## 安全與工程規則
- 所有命令標明在哪台機器執行（開發 PC/Jetson/MCU）。
- 確認供電、電平與手冊後才接線；確認 stop path 前不讓車輪落地運行。
- 每個馬達測試都要限速、限時、有 stop command、timeout 和失敗處置。
- 不可把 Gazebo truth、模擬 topic 或舊 observer 接入實車控制回路。
- 先確認 ROS 2 distro 與 JetPack 相容性，再決定安裝。
- 保留所有歷史 evidence；不要重跑已耗盡的 Phase 7G runtime slots。
