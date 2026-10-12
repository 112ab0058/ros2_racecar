# 實車 RUNBOOK（WAVE ROVER + Jetson Orin Nano）

更新時間：2026-10-13。硬體細節見 [REAL_CAR_HARDWARE.md](REAL_CAR_HARDWARE.md)，目前狀態見 [../STATUS.md](../STATUS.md)。
每個指令都標明在哪台機器跑。**會讓輪子轉的步驟前面都標了 ⚠️：執行前車輪必須架空（除非該步驟另外說明），旁邊要有人，手放在車子電源鍵旁。**

## 0. 安全規則（摘要，完整版見 CLAUDE.md）

- 任何會讓馬達轉的程式或指令，先確認：限速、限時、怎麼停。
- driver 有 `/cmd_vel` 逾時停車（`cmd_timeout` 0.3 s）、速度上限（`max_speed` 0.05 = 約 10 % PWM）、回授遺失停車（2 s 沒有 T:130 回應）、低電壓拒絕指令（< 10.5 V）、關閉時送停止。
- 停車方法，由快到慢：車子電源鍵 → 在 launch 的終端機按 Ctrl-C → 停止送 `/cmd_vel`（0.3 s 內停）→ ESP32 自己約 3 s 沒收到指令會停。
- stop 與 timeout 沒在架空狀態驗證前，不在地面自主行駛；repo 裡沒有自主行駛的 launch。
- **舊的 `wave_rover_driver`（`~/ros2_ws`）和新的 `wro2026_base_waverover` 不能同時跑**（新 driver 和 `test_motion` 會檢查並拒絕啟動）。

## 1. 部署與 build

Windows（開發機，PowerShell）：

```powershell
cd $env:USERPROFILE\ros2_racecar\src
scp -r wro2026_base_waverover wro2026_real user2@192.168.0.15:racecar_ws/src/
```

（先在 Jetson 刪掉 `~/racecar_ws/src/` 裡的舊版本再複製；`~/racecar_ws` 不是 git repo。）

Jetson：

```bash
cd ~/racecar_ws
source /opt/ros/jazzy/setup.bash
colcon build
colcon test && colcon test-result --verbose      # 不需要硬體
source install/setup.bash
```

不要同時 source `~/ros2_ws/install/setup.bash`（舊 driver 的 workspace）和 `~/racecar_ws/install/setup.bash`，避免搞混是哪一個 driver。

## 2. 啟動與停止

Jetson，前景執行（Ctrl-C 停止，log 會出現 `shutdown stop sent: ok`）：

```bash
pgrep -af 'lib/wave_rover_driver/driver|lib/wro2026_base_waverover/driver' || echo "沒有 driver 在跑"
ros2 launch wro2026_real bringup.launch.py transport:=http host:=192.168.0.16     # Wi-Fi
ros2 launch wro2026_real bringup.launch.py                                        # UART /dev/ttyTHS1（預設）
```

- 只想看資料、絕對不讓輪子轉：加 `max_speed:=0.0`（任何 `/cmd_vel` 都只會送出停止）。
- 在背景跑 launch 時，Ctrl-C 對應的是對 **`ros2 launch` 本身的 PID** 送 SIGINT（`kill -INT <pid>`）。非互動 shell 的背景工作會繼承「忽略 SIGINT」，送給包在外面的 shell 不會有效果（2026-10-13 驗證時踩過）。
- 啟動後檢查：

```bash
ros2 topic info /cmd_vel                       # Publisher count 應該是 0（除非你開了 teleop）
ros2 topic hz /battery                         # 約 4.8 Hz（HTTP）
ros2 topic echo --once /diagnostics            # level 0 = OK，message 說明原因
ros2 topic echo /wave_rover/sent_cmd           # 每一筆送出的 T:1
ros2 run tf2_ros tf2_echo odom base_link
```

`/diagnostics` 的 ERROR：`no T:130 reply ...`（回授遺失，輪子已停）或 `battery ... non-zero commands refused`（電池 < 10.5 V）。WARN：等待第一筆回授、電池 < 11.0 V、IMU 沒更新。

## 3. 架空目視測試（HTTP，現在就可以做）

1. 車子架空，四輪離地、固定車體，電池 ≥ 11.0 V。
2. ⚠️ Jetson：

```bash
ros2 run wro2026_base_waverover test_motion --transport http --host 192.168.0.16 --speed 0.05
```

   每個動作 ≤ 1 s、間隔 ≥ 2 s；結果在 `~/wave_rover_logs/motion_<time>_summary.json`。
3. 目視確認：

| 步驟 | 指令 | 應該看到 |
|---|---|---|
| B2a 前進 | L=+0.05、R=+0.05 | 四輪都往前轉 |
| B2b 後退 | L=-0.05、R=-0.05 | 四輪都往後轉 |
| B2c 原地左轉 | L=-0.05、R=+0.05 | 左側往後、右側往前 |
| B2d 原地右轉 | L=+0.05、R=-0.05 | 左側往前、右側往後 |
| 每次之後 | 停止 | 輪子完全停住 |

4. 0.05 都沒轉：改 `--speed 0.1` 再看一次（`test_motion` 上限 0.1）。結果記到 REAL_CAR_HARDWARE.md。

## 4. 明天接 UART 之後（照順序，任何一步失敗就停下來）

接線（Orin 和車子都先關電源）：Orin pin 8（TX）→ 車頂 RX、pin 10（RX）← 車頂 TX、pin 6 GND ↔ GND。**不要接 5V**（Orin 的 5V 腳是輸出，不能和車子的 5V 互接）。接之前再對一次 Waveshare 的腳位圖；兩邊都是 3.3 V 邏輯。

Jetson：

```bash
source /opt/ros/jazzy/setup.bash && source ~/racecar_ws/install/setup.bash
```

1. UART 唯讀檢查（不寫入任何資料），要 0 FAIL：

```bash
ros2 run wro2026_base_waverover check_serial
```

2. 不動輪子的 serial 測試（只檢查啟動送停止、回授、關閉送停止）：

```bash
ros2 run wro2026_base_waverover test_motion --transport serial --port /dev/ttyTHS1 --startup-only
```

3. 確認上一步 B1 的 `t130_count` 大於 0。是 0 代表沒收到回授：先檢查 TX/RX 是否接反，**不要做下一步**（新 driver 沒收到 T:130 也會拒絕所有非零指令）。
4. 不動輪子的 bringup，看 topic（另開終端機跑第 2 節的檢查指令，`/diagnostics` 要 OK）：

```bash
ros2 launch wro2026_real bringup.launch.py max_speed:=0.0
```

5. ⚠️ 架空低速動作測試（同第 3 節的目視表）：

```bash
ros2 run wro2026_base_waverover test_motion --transport serial --port /dev/ttyTHS1 --speed 0.05
```

6. ⚠️ 架空手動遙控（兩個終端機）。teleop 預設 0.5 會被 `max_speed` 夾到 0.05；按住按鍵時鍵盤重複的初始延遲可能超過 0.3 s，所以用 `cmd_timeout:=0.5`。要停就按 `k` 或 Ctrl-C：

```bash
ros2 launch wro2026_real bringup.launch.py max_speed:=0.05 cmd_timeout:=0.5
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 5. 量測（填 UNKNOWN）

量到的值寫進 `src/wro2026_base_waverover/config/wave_rover.yaml` 和 REAL_CAR_HARDWARE.md。

### 5.1 輪距 `track_width`、輪徑

- 用尺量左右輪接地點中心的距離（前後軸各量一次，取平均）→ `track_width`。目前 0.125 m 是韌體常數，不是實測。
- 用游標卡尺量輪子外徑 → 記到 REAL_CAR_HARDWARE.md（之後換算 `pwm_to_mps` 用）。
- 4WD 滑移轉向的「有效輪距」通常比幾何輪距大，等 `pwm_to_mps` 量好後，可在地面原地旋轉比較指令角速度和 IMU 角速度再修正（屬於地面測試，要先完成第 4 節）。

### 5.2 `pwm_to_mps`（每 1.0 輪指令對應多少 m/s）

方法 A（架空，先做）：

1. 輪子側面貼一條膠帶當記號。
2. ⚠️ 用第 4 節第 6 步的遙控，以固定指令（例如 L=R=0.05）讓輪子轉約 5 s，數轉了幾圈 n，用 `ros2 topic echo /wave_rover/sent_cmd` 的時間算出非零指令持續時間 T。
3. 輪子表面速度 = n × π × 輪徑 / T；`pwm_to_mps` = 表面速度 / 0.05。
4. 用 0.05 和 0.1 各量 3 次，看是否線性（低 PWM 可能有死區）。

方法 B（地面，H2 完成之後才做）：在平坦、淨空的地面，⚠️ 用遙控直線前進約 2 s，量實際距離 d，`pwm_to_mps` = d / (T × 指令值)。架空和地面的值可能不同（負載），以地面值為準。

### 5.3 IMU 方向 `imu_yaw_rate_sign`

不用動輪子：driver 開著（`max_speed:=0.0`），用手把車從上往下看**逆時針**轉，`ros2 topic echo /imu/data` 的 `angular_velocity.z` 應該是正的；如果是負的，設 `imu_yaw_rate_sign: -1.0`。

注意（2026-10-13 實測）：靜止時 gz 雜訊很大（30 筆標準差 5.1 deg/s，範圍 -14.7～+13.9 deg/s），gx/gy 只有約 0.2 deg/s；韌體自己的 yaw（T:126 的 `y`）卻很穩定。odom 目前用 gz 積分，行進中的 yaw 可能漂得很快，之後可考慮改用 `y` 的差值。

## 6. 疑難排解

| 現象 | 可能原因 / 處理 |
|---|---|
| `another WAVE ROVER driver is already running` | 舊 driver 或另一個新 driver 還在跑：`pgrep -af driver` 找到後 Ctrl-C 它 |
| `/diagnostics` ERROR `no T:130 reply` | HTTP：ESP32 IP 變了（DHCP）或 Wi-Fi 斷了；serial：TX/RX 接反或沒接 GND |
| `send ... failed` 警告一直出現 | HTTP 逾時；確認 `ping 192.168.0.16`，必要時在路由器設定固定 IP |
| `/odom` 不會前進 | 正常：`pwm_to_mps` 還是 0（UNKNOWN） |
| `battery critical` | 電池 < 10.5 V，充電（12.6 V 變壓器，可邊充邊用） |
