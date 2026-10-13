# 實車硬體盤點（H0）

更新時間：2026-10-13
狀態：過渡測試車已可由 Orin 透過 HTTP 下指令；序列埠接線預計 2026-10-14 完成。
2026-10-13 02:28（+08:00）：driver 已搬進 repo（`src/wro2026_base_waverover`），新 driver 以 HTTP 做過只讀驗證（見下方「已驗證（2026-10-13 02:28）」）。Jetson 的 `~/ros2_ws/src/wave_rover_driver` 保留為備份，兩個不能同時跑。操作步驟見 [RUNBOOK.md](RUNBOOK.md)。
規則：未經實物或文件確認的項目一律標 `UNKNOWN`。

## 系統一覽

```text
Jetson Orin Nano (Ubuntu 24.04, ROS 2 Jazzy)
   │  wro2026_base_waverover（原 wave_rover_driver）: /cmd_vel → {"T":1,"L":..,"R":..}
   │  ├─ 目前：HTTP  http://<ESP32 IP>/js?json=...（Wi-Fi）
   │  └─ 規劃：UART  /dev/ttyTHS1 115200（40-pin：pin 8 TX、pin 10 RX、pin 6 GND）
   ▼
Waveshare WAVE ROVER（ESP32 驅動板 + 4 顆馬達，差速、無 encoder）
```

## Jetson

| 項目 | 值 | 來源 |
|---|---|---|
| 型號 | Jetson Orin Nano Developer Kit，RAM 7.4 GiB | `free -h`、外盒 |
| 外殼 | Waveshare 金屬殼，兩支 Wi-Fi 天線；40-pin 在殼內，需拆前蓋 | 實物 |
| OS | Ubuntu 24.04.4 LTS（minimized），kernel 6.8.12-1021-tegra | `/etc/os-release`、`uname -r` |
| L4T | R39.2.1（build 2026-08-07） | `/etc/nv_tegra_release` |
| JetPack 套件版本 | UNKNOWN（`dpkg -l nvidia-jetpack` 無輸出） | |
| ROS 2 | Jazzy，`ros-base` + `ros-dev-tools`（2026-10-11 安裝） | apt |
| 儲存 | microSD 56 GB（`/dev/mmcblk0p1`） | `df -h` |
| 電源模式 | MAXN_SUPER（mode 2），不得更改 | `nvpmodel -q` |
| 待機溫度 | CPU/GPU 約 47 °C | thermal zones |
| 使用者 / 主機名稱 | `user2` / `112ab-jno` | |
| 遠端 | NetBird 0.79.0（112ab-jno.netbird.cloud）；區網 DHCP，IP 會變 | `netbird status` |
| 電源輸入 | DC 9–20V；目前用原廠變壓器，車上供電方案 UNKNOWN | |

## 底盤：Waveshare WAVE ROVER

| 項目 | 值 | 來源 |
|---|---|---|
| 運動學 | 差速，4WD，可原地旋轉 | 實物 |
| 下位機 | ESP32 驅動板，JSON 指令，115200 baud | Waveshare wiki、實測 |
| Encoder | **無**。T:130 回授的 L/R 在送出 T:1 期間仍為 0 | 2026-10-13 實測 |
| IMU | 有，T:126 回傳 r/p/y（度）、ax/ay/az（mg）、gx/gy/gz（deg/s） | 韌體原始碼 + 實測（靜止 az ≈ 980） |
| 輪距 | 韌體常數 0.125 m；Waveshare 正視尺寸圖：輪寬 42.5 mm、兩輪內側之間標 81.6 mm，推算輪心距約 0.124 m，和韌體常數一致；實測 UNKNOWN | ugv_base_general 原始碼、Waveshare 尺寸圖 |
| 外型（正視圖） | 含天線高 100 mm、車身頂部 88.5 mm、底盤離地 33.7 mm | Waveshare 尺寸圖 |
| 輪徑 | UNKNOWN | |
| 電池 | 3 × INR18650 2.5 Ah 串聯（3S，滿電 12.6 V），電池座在底板下（拆 4 顆黑色內六角螺絲） | 實物 |
| 充電 | 12.6 V 2A 變壓器，可邊充邊用；剛裝電池需先插充電器喚醒保護板 | 實測 |
| 上位機介面 | 車頂 40-pin 母座（5V、GND、TX、RX、SDA、SCL，對應 Raspberry Pi 腳位 2–10） | 實物絲印 |
| 控制板 USB | 在車殼內，需整台拆開，目前不用 | Waveshare wiki |
| Wi-Fi | AP 模式 SSID `UGV`（192.168.4.1）；目前以 STA 模式連家中網路（DHCP） | OLED、實測 |
| ESP32 MAC | 30:76:F5:7D:DD:44 | OLED |
| 停車保護 | ESP32 約 3 s 沒收到指令自動停 | Waveshare wiki |

### 指令（只允許使用這些）

| 指令 | 用途 |
|---|---|
| `{"T":1,"L":x,"R":y}` | 左右輪 PWM 比例，-0.5～0.5 |
| `{"T":130}` | 底盤回授（含電池電壓） |
| `{"T":126}` | IMU |
| `{"T":131,"cmd":0/1}` | 持續回授開關（driver 目前不用） |

## 附帶配件（目前不用）

- 5 線杜邦線與 Waveshare Jetson Nano Adapter (C)：給 40-pin 外露的主機接車頂 40-pin，會一起接 5V。Orin 不使用，因為 Orin 2/4 腳 5V 是輸出，不能和車子 5V 互接。
- 黑色固定板、銅柱：把主機鎖到車頂，等電源方案確定後再裝。

## 已驗證（2026-10-13，車輪架空，max_speed 0.05）

- 啟動只送停止、前進/後退/左轉/右轉各約 0.9 s、`/cmd_vel` 中斷 0.306 s 內送停止、關閉 driver 時送停止：全部 PASS。
- 輪子實際轉向：2026-10-13 11:14 用新 driver（`wro2026_base_waverover`，HTTP，max_speed 0.05，架空）跑 `test_motion`，B1–B4 全部 PASS（`/cmd_vel` 中斷後 0.306 s 送停止，電池 11.99 V），使用者目視確認前進、後退、原地左轉、原地右轉的方向正確，每次都有停住。紀錄：Jetson `~/wave_rover_logs/motion_20261013_111415.csv`。

## 已驗證（2026-10-13 02:28，新 driver `wro2026_base_waverover`，HTTP，只讀）

條件：`ros2 launch wro2026_real bringup.launch.py transport:=http host:=192.168.0.16 max_speed:=0.0`，沒有任何 `/cmd_vel` publisher（啟動前後都確認 Publisher count 0）。只送了 `{"T":1,"L":0,"R":0}`、`{"T":130}`、`{"T":126}`。Log：Jetson `~/wave_rover_logs/verify_2_5_20261013_022804/`。

| 項目 | 結果 |
|---|---|
| 啟動後只送停止 | `/wave_rover/sent_cmd` 收到 170 筆，全部 `{"T":1,"L":0,"R":0}`、全部送達 |
| `/battery` | 約 4.8 Hz，12.12 V，frame `base_link` |
| `/imu/data` | 約 4.8 Hz，frame `imu_link` |
| `/diagnostics` | 1.0 Hz，level OK，回授延遲約 0.15 s |
| `/odom` | 20 Hz；沒有指令時相隔 33.7 s 的兩筆都是位置 0、角度 0 |
| TF | `odom → base_link`（driver）→ `imu_link`、`camera_link`（robot_state_publisher）可以查到 |
| 關閉送停止 | 對 `ros2 launch` 送 SIGINT：log `shutdown stop sent: ok`，兩個 process 正常結束；之後直接讀 T:130 為 L=0、R=0 |

### 新發現

- **IMU gz 靜止時雜訊很大**：直接讀 30 筆 T:126，gz 平均 0.05 deg/s、標準差 5.1 deg/s、範圍 -14.7～+13.9 deg/s；gx、gy 標準差只有約 0.2 deg/s。韌體融合後的 yaw（`y`）在同一段約 3 秒內只變 0.16°。odom 在沒有指令時固定不動，但行進中用 gz 積分的 yaw 可能漂得快（之後可考慮改用 `y`）。
- HTTP 模式下，沒變的停止指令實際約 1.7 Hz 重送（設定每 0.5 s，加上 HTTP 延遲與回授輪詢）。
- T:130 回授的 `L`/`R` 靜止時為 0（與先前一致）；電池 12.12 V。
- 同日 10:46 再做一次只讀驗證時電池 11.08 V（車子整晚開著，約 8 小時降 1 V）；接近 11.0 V 警告門檻，測試前要充電。

## 對里程碑的影響

- **H3 encoder odometry 在這台車上做不到**：WAVE ROVER 沒有 encoder。過渡期只能用 IMU yaw + 指令速度做 dead reckoning，精度有限，不可作為 Nav2 的主要 odom。比賽車選型時 encoder 為必要條件。
  - 2026-10-13：`wro2026_base_waverover` 已實作這個 dead reckoning（線速度 = 送出的輪指令 × `pwm_to_mps`，角速度 = IMU gz）。`pwm_to_mps` 還沒量（預設 0，所以 odom 只會轉向不會位移）。
- 底盤以上的節點仍只依賴 `/cmd_vel`、`/odom`、`/imu`、TF，不得依賴 WAVE ROVER 的原地旋轉能力。

## UNKNOWN 清單

1. 實測輪距、輪徑、最大線速度（PWM 比例 → m/s 的對應，driver 參數 `pwm_to_mps`）。量法見 [RUNBOOK.md](RUNBOOK.md) 第 5 節。
2. 車上替 Orin 供電的方式（行動電源 PD 20V + 誘騙線為候選，未購買）。
3. 相機型號與安裝位置。
4. LiDAR 型號（尚未安裝）。
5. 比賽車的底盤、馬達、encoder、MCU。
6. 急停：目前只有車子電源鍵與 ESP32 3 s timeout，沒有獨立急停開關。
7. IMU 軸向與 `base_link` 的關係（`imu_yaw_rate_sign`）、IMU 安裝位置。
