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

**Windows（PowerShell）**：拿新程式、複製到 Orin

```powershell
cd $env:USERPROFILE\ros2_racecar
git status                       # 確認沒有沒存的修改
git pull
ssh user2@192.168.0.15 "rm -rf ~/racecar_ws/src/wro2026_base_r550"
scp -r src\wro2026_base_r550 user2@192.168.0.15:racecar_ws/src/
```

先刪再複製：直接覆蓋時，舊版有、新版沒有的檔案會留下來，新舊混在一起。

**Orin**（`ssh user2@192.168.0.15` 登入後）：

```bash
cd ~/racecar_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/wro2026_base_r550 install/wro2026_base_r550   # 清掉舊的 build 產物
colcon build --packages-select wro2026_base_r550
source install/setup.bash
colcon test --packages-select wro2026_base_r550 && colcon test-result --verbose   # 不需要硬體
```

沒清 `build/`、`install/` 的話，`ros2 run` 可能出現 `StopIteration`（舊版的登記資料殘留，2026-10-16 遇過）。

每開一個新終端機都要重新 `source` 這兩個 `setup.bash`。

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
| 裝置 | `/dev/ttyACM0`（插拔順序可能改變號碼） |
| **固定名稱（用這個）** | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00` |
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
