> 來源：WSL `~/ros2_racecar/docs/HARDWARE_INVENTORY.md`（2026-10-09 建立，未追蹤），2026-10-13 搬進 repo。內容描述的是**最終比賽車**（Ackermann），不是目前的過渡測試車 WAVE ROVER；WAVE ROVER 見 [REAL_CAR_HARDWARE.md](REAL_CAR_HARDWARE.md)。除了交接文件路徑已更新為封存後的位置外，內文未修改。

# 實車硬體盤點（H0 草稿）

狀態：DRAFT，尚未經使用者逐項確認。未確認項目一律標 UNKNOWN，不得用猜測補齊。
建立日期：2026-10-09

## 來源
- 使用者提供的「整體系統分層」表（2026-10-09）。
- `src/wro2026_sim/config/phase7/robot_contract.yaml` 的 `target_v550_platform`。
- `docs/SYSTEM_ARCHITECTURE.md` 的 Target Physical Stack。

## 系統分層（使用者提供）

| 層級 | 內容 | 用途 |
|---|---|---|
| 場地層 | 160×160 cm 場地、中央島、避障區、停車區 | 測試環境 |
| 車體層 | 阿克曼轉向車、馬達、輪胎、轉向機構 | 移動平台 |
| 底層控制層 | STM32、PWM、PID、Encoder、UART/CAN | 控制馬達、轉向、讀車體狀態 |
| 感測層 | LiDAR、Orbbec Depth Camera、IMU、Encoder | 環境與車體資料 |
| 運算層 | 主控板、NPU/GPU、ROS 2 | 高階運算、AI、導航 |
| 自駕軟體層 | SLAM、Nav2、避障、路徑規劃 | 定位、建圖、導航 |
| 視覺化層 | Foxglove、RViz、rosbag | 展示、除錯、記錄 |

## 硬體表

| 項目 | 已知 | 待確認 |
|---|---|---|
| 主控板 | Jetson Orin（交接文件） | Nano/NX/AGX、RAM、carrier board、JetPack/L4T |
| OS / ROS | ROS 2 | Ubuntu 版本、ROS 2 distro、RMW/DDS |
| 底盤 | Ackermann；平台名 V550（repo）；車身 0.280×0.190 m，設計包絡 0.300×0.210×0.230 m（repo） | 品牌/說明書、驅動輪（後驅？）、有無差速器 |
| 驅動馬達 | 有 | 型號、有刷/無刷、額定電壓/電流、減速比 |
| 轉向 | 有轉向機構 | 舵機或馬達+回授、型號、PWM 規格、最大轉向角、有無轉向角回授 |
| Encoder | 有 | 型號、CPR、x1/x2/x4、安裝在馬達或輪軸 |
| 馬達 driver / ESC | UNKNOWN | 型號、介面、電壓/電流、保護 |
| MCU | STM32；PWM + PID + Encoder | 晶片/開發板型號、韌體來源、與 Jetson 用 UART 還是 CAN（兩者擇一或都有）、電平 3.3V? |
| LiDAR | 有 | 型號（RPLIDAR/YDLIDAR/…）、介面、供電、掃描頻率 |
| 相機 | Orbbec Depth Camera | 型號（Astra/Gemini/Femto…）、USB 版本 |
| IMU | 有 | 型號、在 STM32 上還是直接接 Jetson、介面 |
| 電源 | UNKNOWN | 電池類型/電壓/容量、DC-DC、Jetson 與馬達是否分開供電、保險絲 |
| 安全 | UNKNOWN | 急停/快速斷電、通訊中斷時 MCU 行為（watchdog/timeout） |

## 待量測尺寸
見 `src/wro2026_sim/config/phase7/v550_geometry_measurement_template.md`：
軸距、前/後輪距、輪徑、最大內/外轉向角、前後懸、LiDAR/相機/IMU 相對 `base_link` 的位置。目前全部 BLOCKED。

## 預定資料流（docs/SYSTEM_ARCHITECTURE.md）

```text
Encoder / IMU → STM32 (PID, PWM) ⇄ UART/CAN ⇄ Jetson (ROS 2)
                                              ├─ /odom /imu /scan /tf
                                              ├─ SLAM / AMCL → Nav2
                                              └─ /cmd_vel → Ackermann adapter → STM32
LiDAR, Orbbec → USB → Jetson
Foxglove / RViz / rosbag ← Jetson
```

注意：Nav2 預設輸出 `/cmd_vel`（Twist），Ackermann 車需要把 (v, ω) 轉成 (v, 轉向角)，
`robot_contract.yaml` 的 `ackermann_command_adapter` 目前為 null，需要軸距才能實作。

## 最低風險下一步
1. 盤點：補齊上表；拍照 Jetson 標籤、STM32 板、driver、電池/接線。
2. Jetson 唯讀盤點命令（見 `docs/archive/handoff_2026-10-09/FIRST_DAY_CHECKLIST.md`）。
3. 確認 STM32 韌體是否已存在、通訊協定（封包格式、鮑率/CAN ID）。
4. 確認急停與通訊 timeout 停車後，才做車輪架空低速測試（H2）。
