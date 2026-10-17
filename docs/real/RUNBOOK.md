# R550 操作手冊（RUNBOOK）

更新時間：2026-10-16。每個指令都標明在哪台機器跑。原理說明見 [R550_STUDY_NOTES.md](../research/R550_STUDY_NOTES.md)。

## 0. 安全規則（摘要，完整版見 CLAUDE.md）

- 任何會讓馬達轉的程式，執行前先確認：限速、限時、怎麼停。
- 第一次讓輪子轉一定要**架空**（輪子離地），手放在 Motor 開關旁邊。
- 目前 `wro2026_base_r550` **只讀**，不會送任何指令給 STM32。
- **`flag_stop` 不代表 Motor 開關**（開關在 OFF 時也讀到 0），不能用它判斷馬達會不會動。

## 1. 程式放在哪、怎麼流動

```text
Claude 的工作區 ──push──▶ GitHub（正本） ──pull──▶ Windows ──scp──▶ Orin ~/racecar_ws（不是 git repo）──build──▶ 執行
```

## 2. 部署與 build

**Windows**：在 repo 資料夾的 `tools\real\` 裡雙擊 `deploy.cmd`，或在 PowerShell 執行：

```powershell
cd $env:USERPROFILE\ros2_racecar
tools\real\deploy.cmd
```

它會依序做三件事，中間只會問一次 Orin 密碼：

1. `git pull --ff-only`：Windows 上有沒存的修改就停下來，不會亂合併。
2. 用 `tar` 把 4 個實車 package（`wro2026_base_r550`、`wro2026_lidar_n10p`、`wro2026_description`、`wro2026_bringup`）打包，透過**一條 ssh** 送到 Orin 的 `~/racecar_ws/.deploy`。`wro2026_sim` 不會送。
3. 在 Orin 上執行 `tools/real/orin_build.sh`：
   - 先刪掉舊的 `src/`、`build/`、`install/`，再放新的。直接覆蓋的話，舊版有、新版沒有的檔案會殘留；舊的 build 產物會造成 `StopIteration`（2026-10-16 遇過）。
   - 最後跑 `colcon build`。

最後顯示 `OK` 就是成功。顯示 `FAILED` 時，往上看錯誤訊息。

**Orin 上跑測試**（不需要硬體，想確認時再跑）：

```bash
cd ~/racecar_ws && source install/setup.bash
colcon test --packages-select wro2026_base_r550 wro2026_lidar_n10p wro2026_description wro2026_bringup && colcon test-result --verbose
```

每開一個新終端機都要先 `source /opt/ros/jazzy/setup.bash` 和 `source ~/racecar_ws/install/setup.bash`。

## 3. 開機順序

1. **Motor 開關放 OFF**（紅色，藍色電源端子旁邊）。
2. 打開電源開關（黑色，藍色端子另一側），蜂鳴器短響一聲。
3. **10 秒內不要碰車**：STM32 在取 IMU 零點。
4. 再把「STM32通信线」（C30D 上印 `UART3` 的孔）的 USB-A 插到 Orin。

**順序不能反**：車子關著就插 USB，Orin 的 5 V 會從 USB 反向供電讓 STM32 開機，但量到的電池電壓接近 0，蜂鳴器會每秒響（低電壓警報）。

**蜂鳴器**：開機響一聲 = 正常；10.5 V 響一聲 = 電量偏低；低於 10 V 每秒響一次 = 電壓過低（或上面說的反向供電）。

## 4. 裝置名稱

| 項目 | 值 |
|---|---|
| USB 轉串口晶片 | CH9102（`lsusb`：`1a86:55d4 QinHeng`），Linux 內建 CDC-ACM driver |
| 裝置 | `/dev/ttyACM*`（插拔順序會改變號碼，不要用） |
| **STM32 固定名稱** | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00` |
| **光達固定名稱** | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00`（同一種晶片，只能靠序號分） |
| 權限 | user2 在 `dialout` 群組，可以直接讀 |

## 5. 只聽 STM32（不需要 ROS node）

**Orin**：

```bash
ros2 run wro2026_base_r550 listen --port /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00 --seconds 10 --raw-out ~/r550_capture.bin
```

正常的樣子（2026-10-16）：約 20 Hz、`bad_frames=0`、靜止時 `vx`、`wz` 為 0、`acc` 的 z 約 +9.81、`battery` 和 OLED 一樣。

看原始位元組：`xxd ~/r550_capture.bin | head`（每個封包 `7b` 開頭、`7d` 結尾、24 bytes）。

## 6. ROS node（只讀）

**Orin**：

```bash
ros2 launch wro2026_base_r550 feedback.launch.py port:=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

另開一個終端機（記得 `source`）檢查：

```bash
ros2 topic list                      # 應該有 /odom /imu/data /battery /diagnostics /tf
ros2 topic hz /odom                  # 約 20 Hz
ros2 topic echo /battery --once
ros2 topic echo /diagnostics --once
```

停止：在跑 launch 的終端機按 `Ctrl+C`。

### 全部一起開（一個終端機）

**Orin**：

```bash
ros2 launch wro2026_bringup sensors.launch.py
```

這一行會啟動底盤回授（`/odom`、`/imu/data`、`/battery`）、光達（`/scan`）、URDF（TF）和唯讀的 Foxglove bridge。全部都只讀，馬達不會轉。按 `Ctrl+C` 全部一起停。

- 不要 Foxglove：加 `foxglove:=false`。串口不一樣時：加 `stm32_port:=...`、`lidar_port:=...`。
- 光達串口**一次只能給一個程式**：`listen` 和 node 同時開，會出現 `multiple access on port?`。用 `pgrep -af n10p` 找出來、`pkill -f wro2026_lidar_n10p` 關掉。
- TF 檢查：`ros2 run tf2_ros tf2_echo base_link laser_link` 應該是 `[0.130, 0.000, 0.115]`。
- Foxglove：Windows 瀏覽器開 app.foxglove.dev，連 `ws://192.168.0.15:8765`。bridge 是唯讀的，電腦端不能發 topic。

### 自動檢查（不用開一堆終端機）

**Orin**：

```bash
bash ~/racecar_ws/tools/orin_check.sh            # 約 1 分鐘
bash ~/racecar_ws/tools/orin_check.sh --bag 20   # 另外錄 20 秒 rosbag（錄的時候用手推車）
```

- 沒有在跑 `sensors.launch.py` 的話，腳本會自己開（不含 Foxglove），檢查完再關掉。
- 檢查項目：串口、node、topic 頻率、電池、TF、一筆 `/scan` 的有效點數。
- 結果存在 `~/r550_check/<時間>/`：`report.txt`、`launch.log`、`scan_once.yaml`，加 `--bag` 時還有 `bag/`。
- 全部只讀，不會讓馬達轉，也不用 sudo。

Windows 的 `tools/real/deploy.sh` 是 `deploy.cmd` 的 bash 版，給 Git Bash、WSL、Claude Code 用。

## 7. 充電

- 12.6 V 充電器接車身「充电口」；紅燈充電中、綠燈充滿。
- 建議 10.8 V 以下充電；**充電時不要使用車子**。

## 8. 疑難排解

| 症狀 | 原因 / 處理 |
|---|---|
| `ros2 run` 出現 `StopIteration` | 舊 build 殘留，照第 2 節清掉 `build/`、`install/` 重 build |
| `/dev/serial/by-id/` 是空的 | 插到錯的孔（要 `UART3`）、或線只能充電不能傳資料 |
| `Permission denied` | `groups` 裡沒有 `dialout` |
| `bad_frames` 一直增加 | baud 不對（要 115200）或接觸不良 |
| 一直叫 | 見第 3 節（反向供電或電池低） |
