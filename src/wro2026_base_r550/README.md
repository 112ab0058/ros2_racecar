# wro2026_base_r550

> **協定來源：WHEELTEC 官方文件**「串口通信控制与反馈」V1.0（2024-06-04）與「阿克曼转向角度指令计算」V1.0（2021-10-26），在 R550 資料包的 `1.WHEELTEC ROS机器人通用资料/2.运动底盘的控制与运动学解析/5.串口控制讲解/`。文件裡印出的範例封包都寫成測試，全部通過。**還沒在實車上錄封包驗證**。（最早是依兩份第三方 driver 寫的：[robotverseny/drivers](https://github.com/robotverseny/drivers) `26a63c5`、[shupx/akm_driver](https://github.com/shupx/akm_driver) `075e0f1`。）

WHEELTEC R550 mini akm 的底盤 package。有：協定 module、**只讀**的回授 node `feedback_node`（發 `/odom`、`/imu/data`、`/battery`）、命令列的 `listen` 工具，以及**會讓馬達轉**的 `base_node`（2026-10-17 寫好，**還沒架空驗證**）。

- `feedback_node`、`listen`：只讀，不訂閱 `/cmd_vel`、不寫序列埠（有測試檢查）。`sensors.launch.py` 用的是這個。
- `base_node`：回授 + 指令端。`enable_commands` 預設 **false**（等於只讀）；設成 true 才會把 `/cmd_vel` 送到 STM32。兩個 node 開同一個串口，**不要同時跑**。

## 指令端的安全規則（`command.py` 的 `CommandGate`，每一條都有測試）

每 1/20 秒決定一次要送什麼，第一個成立的規則勝出：

| 規則 | 結果 |
|---|---|
| `enable_commands` 是 false | 不訂閱、不送任何東西 |
| 收到第一個 `/cmd_vel` 之後超過 `max_run_time` 秒（> 0 時） | 送 0，直到重開（測試時用來限時） |
| 超過 `feedback_timeout`（0.5 s）沒收到 STM32 回授 | 送 0 |
| 濾波後的電壓低於 `min_battery_voltage`（10.5 V） | 送 0，回升 0.2 V 才恢復（瞬間掉壓不會觸發） |
| 還沒收過 `/cmd_vel`，或最新一筆超過 `cmd_timeout`（0.3 s） | 送 0 |
| 正常 | 限速 `max_vx` 0.3 / 倒車 0.15 m/s、`max_wz` 1.0 rad/s；vx 加速度限制 0.5 m/s²；保持指令的曲率；\|vx\| < 0.02 m/s 時 wz = 0（阿克曼不能原地轉） |

- 安全停車是**立刻送 0**，不經過加速度限制。
- 單位換算：STM32 以為輪子一圈 26.4 cm，實際 21.5 cm（`vx_scale` 0.814）。`/odom` 乘 0.814，指令就除以 0.814，所以 `/cmd_vel` 0.1 m/s 實際就是 0.1 m/s。
- 按 Ctrl+C 停 node 時，先送 5 個停車封包再關串口。
- 指令封包的 byte 1 **一定送 0**：韌體收到 1、2、3 會進入自動回充／紅外對接模式（`usartx.c`）。有測試檢查。
- 額外發 `cmd_vel_out`（實際送出的速度，實際單位）和 `/diagnostics` 的 `r550_base: command`（目前的狀態）。
- **已知**：node 被強制結束（沒送停車封包）時，STM32 **不會**自己停（韌體沒有指令逾時）。硬體停車就是 **Motor 開關**。

## 內容

| 檔案 | 用途 |
|---|---|
| `wro2026_base_r550/protocol.py` | 指令封包編碼（Twist 模式、Ackermann 模式）、回授封包解碼、XOR checksum、單位換算、從序列位元組流重組封包 |
| `wro2026_base_r550/feedback.py` | 回授 → 里程計積分（中點法，斷線超過 0.5 s 不積分）、幀率統計 |
| `wro2026_base_r550/feedback_node.py` | ROS 2 node `r550_feedback`（只讀）：`/odom` + TF `odom → base_link`、`/imu/data`、`/battery`、`/diagnostics` |
| `wro2026_base_r550/command.py` | 指令端的安全規則 `CommandGate`（純邏輯，沒有 ROS、沒有串口） |
| `wro2026_base_r550/base_node.py` | ROS 2 node `r550_base`：回授 + `/cmd_vel` → STM32（**會讓馬達轉**） |
| `config/r550_base.yaml`、`launch/base.launch.py` | 指令端參數（`enable_commands` 預設 false）和 launch |
| `test/test_command.py` | 每一條安全規則、參數和 yaml 一致、預設不啟用 |
| `wro2026_base_r550/listen.py` | 命令列工具（不需要 ROS）：每秒印一行解碼結果，可存原始位元組 |
| `config/r550_feedback.yaml`、`launch/feedback.launch.py` | node 的參數和 launch |
| `test/test_feedback.py` | 里程計、listen、**只讀保證**（不寫序列埠、不訂閱、不用指令編碼）、參數和 yaml 一致 |
| `test/test_protocol.py` | pytest；包含**官方文件印出的範例封包**，和依規則手算的封包；都不是從 R550 錄下來的 |

## 協定摘要（官方文件）

C30D 的串口 3（中間的 USB-C），115200 baud，8N1。多位元組都是大端（高位在前），負數用二補數。

主機 → STM32，11 bytes：

| byte | 內容 |
|---|---|
| 0 | `0x7B` 幀頭 |
| 1, 2 | 保留位，無意義，送 0 |
| 3–4 | x 目標速度，int16，mm/s |
| 5–6 | y 目標速度（只有全向底盤用） |
| 7–8 | z 目標角速度，int16，rad/s × 1000。**阿克曼也是角速度**，STM32 自己換算成前輪轉角，並檢查轉彎半徑不小於最小轉彎半徑 |
| 9 | byte 0–8 的 XOR（BCC） |
| 10 | `0x7D` 幀尾 |

STM32 → 主機，24 bytes，預設 20 Hz：`0x7B`、flag_stop（0 = 馬達使能）、x/y/z 速度（mm/s、rad/s × 1000）、加速度計 x/y/z 原始值（÷ 1672 = m/s²）、陀螺儀 x/y/z 原始值（÷ 3753 = rad/s）、電壓（mV）、byte 0–21 的 XOR、`0x7D`。

`encode_ackermann_command`（轉向角 × 1000 / 2）只出現在第三方 driver，官方文件沒有，**不要用在 R550**。

## 文件本身的錯誤（寫測試時發現）

- 「後退 300 mm/s」的封包是 `FE D3` = −301，應該是 `FE D4`。
- 「後退 80 mm/s」那一欄的 z 值和文件自己的公式對不起來（大約多 10 倍），測試不用這一欄。
- 1.1 節文字說 z「順時針為正」，但範例和指令表都是**左轉（逆時針）為正**，和 ROS REP-103 一樣。以實車測試為準。

## 未確認（要在實車驗證）

- 錄到的實際封包是不是 24 bytes、電壓欄位和電池相符。
- 速度和 IMU 的正負方向、`base_link` 的定義。
- 序列埠的裝置名稱（USB 轉串口晶片 CH9102 或 CP2102，`lsusb` 確認）。
- ~~STM32 自己有沒有指令逾時停車~~：**沒有**。韌體 `BALANCE/balance.c` 第 219–221 行的「`command_lost_count` 超過 `RATE_100_HZ`（約 1 秒）就把 `Move_X/Y/Z` 歸零」整段被註解掉（2026-10-19 讀原始碼；`usartx.c` 收到指令時仍會把計數歸零，但沒有地方檢查它），STM32 會一直執行最後收到的指令。所以 `base_node` 的 `cmd_timeout` 是唯一的軟體停車；node 被強制結束（`kill -9`、當機）時車子會繼續跑，只能靠 Motor 開關。

研讀筆記見 `docs/research/R550_STUDY_NOTES.md`，操作步驟見 `docs/real/RUNBOOK.md`。

## 第一次接上 R550：只聽不送

**Motor 開關放 OFF**。這一步不會送任何指令，但還是照安全規則做。

1. Windows：把 `real/r550-prep` 分支拉下來，再照 [RUNBOOK 第 2 節](../../docs/real/RUNBOOK.md)把這個 package 複製到 Jetson 的 `~/racecar_ws/src/`，`colcon build`
2. R550 開主電源，等 10 秒（IMU 取零點，不要碰車）
3. 用 USB 線接 **C30D 中間的 USB-C（串口 3）** 到 Orin。**不要接右邊那個（串口 1 是燒錄用的）**
4. Jetson：
   ```bash
   lsusb                          # 找 CH9102 / CP2102 那一行（QinHeng 或 Silicon Labs）
   ls -l /dev/serial/by-id/       # 找到裝置名稱，例如 /dev/ttyACM0 或 /dev/ttyUSB0
   source ~/racecar_ws/install/setup.bash
   ros2 run wro2026_base_r550 listen --port /dev/ttyACM0 --seconds 10 --raw-out ~/r550_first.bin
   ```
   沒權限（Permission denied）就檢查 `groups` 裡有沒有 `dialout`
5. 對照：每秒約 20 幀、`bad_frames=0`、電壓和 OLED 顯示的一樣、`flag_stop`（**2026-10-16 實測：Motor 開關在 OFF 時也是 0，所以它不代表 Motor 開關**）、車子靜止時 `acc` 的 z 約 +9.8
6. 都正常後再開 ROS node：
   ```bash
   ros2 launch wro2026_base_r550 feedback.launch.py port:=/dev/ttyACM0
   ros2 topic hz /odom              # 約 20 Hz
   ros2 topic echo /battery --once
   ```
   Motor OFF、用手推車子往前，`/odom` 的 x 應該增加（第 2 階段的練習）

## 測試

```bash
cd src/wro2026_base_r550 && python3 -m pytest
colcon test --packages-select wro2026_base_r550 && colcon test-result --verbose
```
