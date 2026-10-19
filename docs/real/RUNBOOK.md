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
3. **10 秒內不要碰車**：STM32 在開機第 10 秒取 IMU 零點（gyro 三軸 + accel，只取一個樣本、沒有平均；`imu_task.c`、`ICM20948.c`）。這時車在動，之後 gyro 會一直有偏差、車頭角度會飄。
   - 想重新歸零：車子靜止時**雙擊**C30D 上的使用者按鍵。**不要單擊**：單擊會切換自動回充（`balance.c` `Key()`）。
4. 再把「STM32通信线」（C30D 上印 `UART3` 的孔）的 USB-A 插到 Orin。

**順序不能反**：車子關著就插 USB，Orin 的 5 V 會從 USB 反向供電讓 STM32 開機，但量到的電池電壓接近 0，蜂鳴器會每秒響（低電壓警報）。

**蜂鳴器**：開機響一聲 = 正常；10.5 V 響一聲 = 電量偏低；低於 10 V 每秒響一次 = 電壓過低（或上面說的反向供電）。

### Orin 改用電池供電之後（2026-10-18 起）

接線：`電池 ← 1分2 公頭 ｜ 1分2 母座 ← 車子原本的 DC 線 ｜ 1分2 另一個公頭 → E1151F（2.1 母 → 2.5 公）→ Orin`。
Orin 的電**不經過車子的電源開關**，Orin 開發板也沒有開關，插上 DC 就開機。

**開機**
1. Motor 開關 OFF。
2. 車子電源開關 ON，響一聲，10 秒內不碰車。
3. 插上 Orin 的 DC 頭（E1151F），Orin 開機。STM32、光達的 USB 可以一直插著。

**關機**
1. Orin 上 `sudo shutdown now`，等燈熄、風扇停（約 20 秒）。
2. 拔掉 Orin 的 DC 頭。
3. 車子電源開關 OFF。

**不能做**：車子電源 OFF、Orin 卻開著（STM32 會被 USB 反向供電，蜂鳴器每秒響）。電池低於 10.5 V 就充電（電池約 9 V 自己斷電，Orin 下限 9 V）。充電時先關 Orin。

**第一次接 Orin 前**：用三用電表量 E1151F 的 2.5 公頭，紅棒碰中心、黑棒碰外殼，要讀到 **+10～12.6 V**（正值）。讀到負值或 0 就停下來。

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

Windows 的 `tools/real/deploy.sh` 是 `deploy.cmd` 的 bash 版，給 Git Bash、Claude Code 用。兩個都是先 `git pull`，再啟動 `deploy_send.*`（送檔和 build），所以 pull 下來的新版腳本這一次就會生效（2026-10-19 以前的版本要跑兩次）。

**檢查前**：Orin 上不能留著其他 ROS 程式（舊的 scan.launch、foxglove 等）。`orin_check.sh` 看到「只有一部分在跑」會直接停下來，請先關掉再跑。

## 6b. 架空測試指令端（會讓馬達轉，先跟 Claude 確認）

**準備**：
- 車子放在盒子上，**四個輪子都離地**，旁邊沒有東西會碰到輪子。
- 只需要 STM32 那條 USB；`sensors.launch.py` **不能**同時在跑（兩個程式會搶同一個串口）。
- 手放在紅色 Motor 開關旁邊。

**Orin**（自己在 ssh 終端機跑，它會一步一步問你）：

```bash
bash ~/racecar_ws/tools/orin_air_test.sh
```

| 步驟 | 做什麼 | 應該看到 | 2026-10-19 架空 |
|---|---|---|---|
| 1 | Motor **OFF**，送 0.08 m/s | 輪子不動（確認指令有送出，但馬達沒電） | ✓ |
| 2 | Motor ON，前進 0.08 m/s 3 秒後停止發布 | 往前轉，停止發布後約 0.3 秒逾時、再約 0.8 秒輪子停 | ✓ odom 最高 +0.083 |
| 3 | 倒車 −0.08 m/s | 往後轉，然後停 | ✓ odom 最低 −0.086 |
| 4 | 要求 1.0 m/s | 只比步驟 2 快一點（被限在 0.1 m/s） | ✓ 送出最高 0.100 |
| 5 | 0.08 m/s + wz +0.16（轉彎半徑 0.5 m，比最小轉彎半徑大） | 前輪往**左**轉（確認正負號） | ✓ odom wz 為正 |
| 6 | 只給 wz、vx = 0 | 完全不動（阿克曼不原地轉） | ✓ 送出 0 |
| 7 | 轉動中停掉 node | 立刻停（停車封包） | ✓（只有目視，舊版腳本沒錄 csv） |
| 8 | `max_run_time` 5 秒，指令送 8 秒 | 約 5 秒後停，即使指令還在送 | ✓ 但測的是舊版（從 node 啟動算 11 秒），見下方 |
| 9 | 電壓下限設 13 V | 不動，gate reason = `battery_low` | ✓ |
| 10 | （不測）node 被強制結束時 STM32 會不會自己停 | **不會**：韌體 `balance.c` 的指令逾時被註解掉，STM32 會一直執行最後一個指令。只能靠我們 node 的逾時和 Motor 開關 | — |

2026-10-19 第一輪 Orin 上跑的是 `e3cb0ad` 之前的版本（`max_run_time` 從 node 啟動算）；同晚部署新版後重測，從第一個指令算，**5.00 秒**觸發。

- **限制**：最高 0.1 m/s、每步 2–8 秒、逾時 0.3 秒。`ros2 topic pub` 啟動要 1 秒多，腳本已自動多算 1.5 秒。
- **停止**：`Ctrl+C`（node 會送停車封包），或 Motor 開關 OFF。
- **結果**：`~/r550_airtest/<時間>/report.txt`，還有每一步的 `cmd_vel_out`、`/odom` CSV。

只開 node、自己發指令：

```bash
ros2 launch wro2026_base_r550 base.launch.py enable_commands:=true max_vx:=0.1 max_run_time:=120
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"   # Ctrl+C 停止發布 → 0.3 s 後停車
```

## 6c. 鍵盤遙控（會讓馬達轉；目前只在架空狀態用）

**Orin**（自己在 ssh 跑，一個終端機就好）：

```bash
bash ~/racecar_ws/tools/orin_teleop.sh            # 最高 0.15 m/s、300 秒
bash ~/racecar_ws/tools/orin_teleop.sh 0.1 120    # 最高 0.1 m/s、120 秒
```

腳本會在背景開 `base.launch.py`（指令開啟、限速、限時），前景跑鍵盤 node，按 Ctrl+C 全部一起關。

| 按鍵 | 動作 |
|---|---|
| `w` / `q` / `e`（按住） | 前進 / 前進左轉 / 前進右轉 |
| `x` / `z` / `c`（按住） | 倒車 / 倒車、前輪往左 / 倒車、前輪往右 |
| `s` 或空白鍵 | 停 |
| `+` / `-` | 加速 / 減速（每次 0.02 m/s，0.04 到上限之間） |
| `a` / `d` | 沒有作用（阿克曼不能原地轉） |
| Ctrl+C | 送出 0 之後離開，背景的 base node 也一起關 |

- **放開按鍵就停**：終端機沒有「放開」事件，只有按住時的連發（Windows 預設先等約 0.5 秒，之後每秒約 30 次）。所以 node 的判斷是：還沒開始連發時等 0.6 秒，開始連發後 0.25 秒沒收到按鍵就送 0。之後輪子再約 1 秒停下（0.8 秒是 STM32 減速）。
- **一次只能按一個鍵**：終端機只會連發最後按下的鍵，所以轉彎要用 `q`/`e`，不能 `w`+`a`。
- node 用 timer **固定 20 Hz 發布**，送 0 也照樣發。不用 `ros2 topic pub`，因為它曾經停頓超過 0.3 秒，觸發逾時（2026-10-19）。
- **限制**：最高 0.15 m/s（腳本不接受更高）、轉彎半徑 0.5 m、逾時 0.3 秒、限時預設 300 秒。
- **停止**：放開按鍵、`s`、空白鍵、Ctrl+C，或把 Motor 開關扳到 OFF。
- **結果**：`~/r550_teleop/<時間>/drive.csv`，一個檔案記三個 topic，每列是 `t`（收到的時間，秒）、`topic`、`vx`、`wz`：`cmd_vel`（按鍵送出的）、`cmd_vel_out`（實際送給 STM32 的）、`odom`（輪子）。放開到停車 = `cmd_vel` 第一個 0 → `odom` 的 vx 回到 0。另有 `base.log`、`record.log`。

## 6d. 校正錄製：里程計、IMU（只讀，馬達不會轉）

`orin_calib.sh` 會自己開 `sensors.launch.py`（沒在跑的話），用 `sensor_log_node` 錄 `/odom`、`/imu/data`，最後用 `analyze_calib.py` 算結果。不會發 `/cmd_vel`；`base_node` 在跑時會拒絕執行。

| 測試 | Orin 指令 | 你要做什麼 | 算出什麼 |
|---|---|---|---|
| 靜止 | `bash ~/racecar_ws/tools/orin_calib.sh static 60` | 什麼都不做（Claude Code 可以自己跑） | gyro 偏差和雜訊、accel 雜訊、哪個軸朝上、靜止時 odom 會不會飄 |
| 直走 | `bash ~/racecar_ws/tools/orin_calib.sh straight` | Motor OFF；先靜止 3 秒，沿捲尺**用手推**車直走（例如 2 m），按 Enter，輸入捲尺距離 | 新的 `vx_scale` |
| 轉圈 | `bash ~/racecar_ws/tools/orin_calib.sh turn` | Motor OFF；先靜止 3 秒，把車**往前推、沿著往左的弧線走**（從上面看逆時針，半徑 ≥ 0.5 m，像在開車），**不要原地轉**（後輪會滑、`wz_scale` 會錯），按 Enter，輸入圈數（例如 0.25） | gyro z 正負號（驗證 `imu_link` 方向）、新的 `wz_scale`（以 IMU 為準） |

- **結果**：`~/r550_calib/<時間>_<測試>/` 裡的 `odom.csv`、`imu.csv`、`analysis_<測試>.txt`。
- 之後在 Windows 也能重算：`python tools\real\analyze_calib.py straight <資料夾> --distance 2.0`
- Orin 還沒上車時線會拉到：推的距離以線夠長為準（1 m 也可以，誤差會比較大）。
- 推的時候輪子要**滾動**。Motor OFF 時馬達經過減速齒輪會有點緊，如果輪子推不動、在地上滑，那次數據不能用。
- 量距離：在地上貼一條膠帶當起點，對齊後輪軸（車身側面的後輪中心），推完量後輪中心到起點的距離。

## 6e. EKF（只讀，馬達不會轉）

```bash
ros2 launch wro2026_bringup localization.launch.py            # sensors + EKF（+ Foxglove）
bash ~/racecar_ws/tools/orin_check.sh --ekf                   # 自動檢查（Claude Code 可以自己跑）
```

- `localization.launch.py` = `sensors.launch.py`（`publish_odom_tf:=false`）+ robot_localization 的 `ekf_filter_node`。
- 融合：`/odom` 的 vx、vy（= 0，阿克曼不能橫移）+ `/imu/data` 的 vyaw。odom 的 vyaw 等轉圈測試（DETAILED_PLAN 7.3）驗證 `wz_scale` 後再決定要不要加。設定和理由在 `src/wro2026_bringup/config/ekf.yaml`。
- 輸出：`/odometry/filtered`（30 Hz）和 TF `odom → base_link`。**同一個 TF 只能有一個發布者**：用這個 launch 時 feedback 不發；以後開車要用 `base.launch.py publish_odom_tf:=false` 再加 EKF。
- `--ekf` 檢查：`/odometry/filtered` 頻率、`publish_odom_tf` 是不是 False、誰在發 `/tf`、靜止 60 秒前後的位置（不能碰車）。

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
