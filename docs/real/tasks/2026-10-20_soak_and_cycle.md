# Claude Code 任務：啟停循環 + 2 小時長時間測試（2026-10-20）

**全程只讀**：只開 `localization.launch.py`（sensors + EKF）和記錄程式。不開 `base_node`、不發 `/cmd_vel`、不用 sudo、不裝套件、不改 repo、不 commit。馬達不會轉。
預計 **約 2.5 小時**。結果放 Windows `下載\r550_cycle\`、`下載\r550_soak\`，最後整理一份摘要給 Ray 貼回 chat。

## 0. 開始前（Ray 做，做完跟你說「好了」）
- 車放在地上或桌上，**Motor 開關 OFF**；光達 USB、STM32 USB 都插著；Orin 插牆壁電源
- 車子電源開著；開機後 10 秒內沒碰車；接下來 2.5 小時**沒有人碰車或桌子**
- 光達最好對著牆或箱子（量光達雜訊用），周圍不要有人走動

## 1. 部署 + Orin 上的單元測試（約 5 分鐘）
1. `tools\real\deploy.cmd`（或 `bash tools/real/deploy.sh`）。先確認 Orin 上沒有 ROS 程式在跑。
2. Orin：
   ```bash
   cd ~/racecar_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
   colcon test --packages-select wro2026_base_r550 wro2026_lidar_n10p wro2026_description wro2026_bringup
   colcon test-result --verbose
   ```
   包含 ament 的 flake8 / pep257（這兩個在 chat 的環境沒辦法用 ament 跑）。**有失敗就記下來，不要自己改程式**，繼續做下一步。
3. 電池電壓 ≥ 11.3 V 才繼續（`ros2 topic echo --once /battery --field voltage`，要先開 sensors）。低於就停下來跟 Ray 說要先充電。**不要邊充電邊跑**：廠商說明書寫充電時不要使用車子，而且充電器會把電壓撐在 12.6 V 附近，電池那一項數據就沒有意義。馬達關著時只有 STM32 和光達在用電（估計 0.3–0.5 A，2.5 小時約 1 Ah，電池 5.1 Ah），`orin_soak.sh` 低於 11.0 V 會自己停，離電池的保護電壓 9 V 還很遠。

## 2. A：啟停循環 30 次（約 15–20 分鐘）
```bash
bash ~/racecar_ws/tools/orin_cycle.sh 30
```
- 每次：啟動 → 等到 `/odometry/filtered` → 檢查 `/odom`、`/imu/data_raw`、`/scan` → Ctrl+C → 等全部結束 → 有沒有殘留 → 兩個序列埠還在不在。
- 看 `summary.txt`：VERDICT 是不是 PASS。有失敗的循環就看那一圈的 `launch_<i>.log`、`left_<i>.txt`，**找出原因寫進摘要**（不要改程式）。

## 3. B：2 小時長時間測試（120 分鐘）
```bash
nohup setsid bash ~/racecar_ws/tools/orin_soak.sh 120 > ~/soak.out 2>&1 &
```
- 用 nohup + setsid，ssh 斷掉也會繼續跑。**每 10 分鐘左右** `tail -n 3 ~/soak.out` 看一次（每分鐘一行：剩幾分鐘、電池、IMU 筆數）。
- 腳本自己會在這些情況提早停：電池 < 11.0 V、有 node 掛掉。要你手動停：`touch ~/r550_soak/STOP`（下一分鐘內乾淨地停）。
- 提早停了：記下 `REASON=` 和最後的 log。**最多重跑一次**，而且只在原因跟車子無關的時候（例如 ssh 問題）。
- 跑完會自動跑 `analyze_soak.py`，報告在 `~/r550_soak/<時間>/report_soak.txt`。

## 4. C：重複檢查（約 15 分鐘）
1. 把 `~/r550_cycle/<時間>/`、`~/r550_soak/<時間>/` 整個抓回 Windows。
2. 在 Windows 再跑一次 `python tools\real\analyze_soak.py <資料夾>`，**跟 Orin 上的報告逐行比對**，應該一模一樣（ZUPT replay 那行在 Windows 也要能跑，因為 repo 裡有 `src/`）。
3. **獨立驗算**（用你自己寫的小程式，不要用 analyze_soak.py 的函式）：
   - `ekf.csv`：最後和最前面的 yaw 差（度）、位置最大偏移
   - `imu_raw.csv`：訊息數 / 時間跨度 = 頻率；最大間隔
   - `battery.csv`：最前、最後的電壓
   - `resources.csv`：每個 node 的 RSS 最前、最後
   跟報告的數字對得上就寫「驗算一致」，對不上就把兩邊的數字都列出來。
4. 看 `allan.csv`：畫或描述曲線形狀（前段斜率約 −1/2 是白雜訊，最低點是 bias instability）。

## 5. 摘要（給 Ray 貼回 chat）
格式：
```
長時間測試（2026-10-20，r550_cycle/<時間>、r550_soak/<時間>，Claude Code 執行）
- colcon test：通過 / 失敗（哪個測試、錯誤訊息第一行）
- 啟停循環：N 次，成功 N；啟動 min/median/max；關閉 max；殘留 N；序列埠消失 N
- 長時間：實際跑了幾分鐘、REASON
- 頻率 / 間隔：imu_raw、odom、ekf、scan/min
- gyro：Allan ARW（deg/√h）、bias instability（deg/h）、raw 偏差 10 分鐘一格的變化
- ZUPT：學到的偏差、replay 比對、修正後平均
- EKF / odom 漂移：yaw（deg、deg/h）、位置
- 電池：起訖、斜率；溫度最高；每個 node 的 CPU %、RSS 起訖（有沒有洩漏）
- 光達：雜訊中位數 / 95%、valid fraction
- 所有 WARN：原因或推測
- 重複檢查：Windows 重算一致？獨立驗算一致？
```

## 安全
- 全程沒有 `base_node`、沒有 `/cmd_vel`。如果聽到馬達聲、看到任何讓車子會動的跡象：`touch ~/r550_soak/STOP`、Ctrl+C 所有程式，跟 Ray 說。
- 不要 `kill -9` 序列埠程式以外的東西；不要動 `~/ros2_ws`。
