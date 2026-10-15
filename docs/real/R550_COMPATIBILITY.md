# WHEELTEC R550 / LeiShen M10P 相容性調查（B3）

> **2026-10-15 更正**：實車到貨，光達是 **LSLIDAR N10P，不是 M10P**；車型是 **R550 mini akm**；相機是 WHEELTEC C70。本文件第 5 節和 build 結果是針對 M10P，要依 N10P 重查（N10P 廠商頁：UART 460800 bps + Ethernet）。實物盤點見 [R550_HARDWARE.md](R550_HARDWARE.md)。

調查日期：2026-10-13（R550 還沒到貨）。目標環境：Jetson Orin Nano、Ubuntu 24.04、ROS 2 Jazzy。
可信度標記：**官方** = 廠商自己的 repo / 頁面；**第三方** = 別人的 repo 或經銷商；**推測** = 由程式或資料推論，沒有直接寫明。

## 1. 結論

| 問題 | 結論 | 可信度 |
|---|---|---|
| WHEELTEC R550 的 ROS 2 程式公開在哪裡 | 沒找到 WHEELTEC 官方的公開 repo。經銷商頁面只寫「ROS 2」、Raspberry Pi 5 版本，沒寫 distro；資料要跟廠商 / 組員拿 | 第三方 |
| WHEELTEC 的 ROS 2 程式能不能在 Jazzy 上跑 | 有第三方把 `turn_on_wheeltec_robot` 移植到 ROS 2 Jazzy 並在 WHEELTEC 阿克曼小車上使用，所以**很可能可以**；但那是別的車型、別人改過的版本 | 第三方 + 推測 |
| STM32 序列協定 | 兩份第三方程式一致：115200 baud、`0x7B` 開頭、`0x7D` 結尾、XOR checksum、指令 11 bytes、回授 24 bytes。兩份都源自 WHEELTEC 原廠 driver（**同源，不算獨立驗證**），而且都不是 R550 / C30D 控制板的資料，一致只代表兩份副本彼此相符。已寫成 `src/wro2026_base_r550`（UNVERIFIED） | 第三方（源自廠商程式）+ 推測（套用到 R550） |
| 阿克曼版收 (v, ω) 還是 (v, 轉向角) | 程式裡兩種都有：ROS 2 移植版預設送 (v, ω)，ROS 1 阿克曼版送 (v, 轉向角 × 1000 / 2)。我們的 R550 韌體用哪一種**未知** | 第三方 + 推測 |
| M10P 的 ROS 2 driver | LeiShen 官方 `Lslidar_ROS2_driver` 分支 `LS-S1_V1.0` 支援 M10P，README 寫有在 Ubuntu 24.04 + Jazzy 測過 | 官方 |
| M10P 在 Jetson 上 build | `lslidar_msgs` 成功；`lslidar_driver` 在 CMake 階段失敗：缺 Boost（本次 build 的實際結果）。依 `CMakeLists.txt` 推測之後還會缺 PCL、pcl_conversions、libpcap。沒有裝，列在第 5 節 | 官方程式的 build 結果 + 推測 |

## 2. WHEELTEC R550 的 ROS 2 程式

| 來源 | 內容 | 可信度 |
|---|---|---|
| [OpenELAB R550 AKM/DiffBot 商品頁](https://openelab.io/de/products/wheeltec-r550-akm-diffbot-rpi5) | Raspberry Pi 5 主機、WheelBoard C30D STM32 控制板、M10P ToF 光達、MG513 馬達 + 500 線 GMR encoder、最高 1.2 m/s、271 × 189 × 151 mm；控制介面列 CAN 與序列埠（視軟體設定）；寫 ROS 2 但沒寫 distro；註明舊版 R550 可能用不同的板子 / 光達 | 第三方（經銷商） |
| [WHEELTEC ROS2-V3.5 (Humble) 常用命令手冊（CSDN）](https://adg.csdn.net/69707309437a6b40336a4b85.html) | WHEELTEC 的 ROS 2 套件是 Humble 版：`turn_on_wheeltec_robot`（底盤、光達、相機）、`wheeltec_nav2`、`wheeltec_slam_toolbox` 等；沒寫 R550、沒寫序列埠參數 | 第三方 |
| [robotverseny/drivers](https://github.com/robotverseny/drivers)（commit `26a63c5`，2025-02-04） | `turn_on_wheeltec_robot`、`lslidar_driver`、`serial_ros2`、`wheeltec_robot_msg` 等，依賴 `ros-jazzy-*`；用在 [JKK Research 的 WHEELTEC Roboworks Ackermann Mini 課程](https://jkk-research.github.io/workshops/wheeltec_real_b/)（寫 Jazzy，也可 Humble） | 第三方 |
| [shupx/akm_driver](https://github.com/shupx/akm_driver)（commit `075e0f1`，2021-03-26） | WHEELTEC 阿克曼車的 ROS 1（Melodic）driver | 第三方 |

風險：WHEELTEC 原廠的 ROS 2 程式是為 Humble（Ubuntu 22.04）寫的；Jazzy 上的移植是第三方做的，而且是不同車型。R550 到貨後，要先拿到**隨車附的原廠資料包**再比對。

## 3. STM32 ↔ 主機序列協定

細節和範例封包見 `src/wro2026_base_r550/README.md` 與 `src/wro2026_base_r550/wro2026_base_r550/protocol.py`。重點：

| 項目 | 值 | 來源 |
|---|---|---|
| baud | 115200 | 兩個來源的 launch / 參數（ROS 2 版 node 的參數預設值寫 9600，但 launch 檔設 115200） |
| 指令 | 11 bytes：`7B`、2 個保留 byte、x / y / z 各 int16 大端（mm/s、mm/s、mrad/s）、XOR、`7D` | 兩個來源一致 |
| 回授 | 24 bytes：`7B`、stop flag、x / y / z 速度、加速度計 x/y/z、陀螺儀 x/y/z、電壓 mV、XOR、`7D` | 兩個來源一致 |
| IMU 量程 | 加速度 ±2 g（16384 LSB/g）、陀螺儀 ±500 deg/s（1 LSB = 0.00026644 rad/s） | 兩個來源一致（加速度換算單位不同：一個換成 g、一個換成 m/s²） |
| 兩個來源不同的地方 | 指令 byte 1：ROS 2 版送 0，ROS 1 阿克曼版送 1（「產品型號」）；ROS 1 版 byte 2 註解「使能」 | — |
| 裝置名稱 | udev 把 CP2102（`10c4:ea60`）或 CH343（`1a86:55d4`）序列晶片對應成 `/dev/wheeltec_controller` | robotverseny 的 `wheeltec_udev.sh` |
| 逾時停車 | 主機端 driver 沒有 `/cmd_vel` 逾時；STM32 端有沒有**未知** | 程式碼裡找不到 |

另外：有第三方專案用 WHEELTEC MiniBalance 控制板韌體，回授是 36 bytes（[DLDLDL13579/Small-crawler-vehicle](https://github.com/DLDLDL13579/Small-crawler-vehicle)）。不同板子或韌體版本的封包長度可能不同，R550 的 C30D 板要實際確認。

## 4. 阿克曼版：(v, ω) 還是 (v, 轉向角)

- ROS 2 移植版：`/cmd_vel`（Twist）送 (vx, vy, ωz)；另有 `akm_cmd_vel` 參數，設成 topic 名稱時改收 `AckermannDriveStamped`、送 (speed, steering_angle × 1000 / 2)。launch 檔預設 `akm_cmd_vel: none`，也就是送 (v, ω)。（第三方）
- ROS 1 阿克曼版：只收 `AckermannDriveStamped`，送 (speed, steering_angle × 1000 / 2)。（第三方）
- 推測：WHEELTEC 的阿克曼韌體收 (v, ω) 時自己換算成前輪轉角；收轉向角時那個「÷ 2」的意義不明（可能是韌體的單位或比例）。**到貨後要架空實測**：送小的 ω，看前輪轉向方向與角度。
- 我們的控制器只送 `/cmd_vel`（m/s、rad/s），所以底盤 driver 如果用 (v, ω) 模式，換車時上層不用改；如果韌體只收轉向角，就在 `wro2026_base_r550` 裡做 (v, ω) → 轉向角的換算（需要軸距）。

## 5. LeiShen M10P 光達

| 項目 | 值 | 來源 / 可信度 |
|---|---|---|
| 官方 driver | [Lslidar/Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver) 分支 `LS-S1_V1.0`，commit `e231396`（2026-01-21），README 標 V5.2.0（2026-01-12） | 官方 |
| 支援的 ROS / Ubuntu | README 列出 Dashing、Eloquent、Foxy、Galactic、Humble、**Jazzy（Ubuntu 24.04）**、Rolling（GitHub 首頁表格只寫 Ubuntu 20.04 + Foxy，以分支裡的 README 為準） | 官方 |
| 型號 | `lidar_type: X10`、`lidar_model: M10P`（同一個 driver 也支援 M10、M10GPS、N10、N10Plus、N301） | 官方 |
| 介面 | 序列埠（`serial_port` 填 `/dev/ttyUSB0` 之類）或網路（`serial_port` 留空，`device_ip` 預設 192.168.1.200、資料埠 2368、設備埠 2369）。R550 上的 M10P 接哪一種**未知** | 官方 / 未知 |
| baud | M10P：**500000**（driver 原始碼 `lslidar_x10_driver.cpp` 依型號設定） | 官方程式 |
| launch | `ros2 launch lslidar_driver lslidar_x10_launch.py`；namespace `x10`；LaserScan topic 名稱 `scan`；馬達開關 `/x10/motor_control`（`std_msgs/Int8`） | 官方 README |

### 5.1 在 Jetson 上 build（只 build，沒有接任何裝置）

位置：Jetson `~/vendor_ws/src/Lslidar_ROS2_driver`（不是 git repo 的 workspace；第三方程式不進我們的 repo）。參考用的 WHEELTEC 程式放在 `~/vendor_ws/reference/`（有 `COLCON_IGNORE`，不會被 build）。

```text
$ cd ~/vendor_ws && colcon build --base-paths src
Finished <<< lslidar_msgs [30.3s]
--- stderr: lslidar_driver
CMake Error ... Could NOT find Boost (missing: Boost_INCLUDE_DIR thread)
Failed   <<< lslidar_driver
```

沒有改任何 driver 的程式。build 在 CMake 找套件的階段就停了，還沒編譯任何 C++，所以**能不能在 Jazzy 上編譯成功仍未知**（README 說有在 Jazzy 測過，這是官方說法，不是我們驗證的）。`CMakeLists.txt` 需要：Boost（thread）、yaml-cpp（已有）、PCL、pcl_conversions、Eigen3（已有）、libpcap。

需要的 apt 套件（**沒有裝，不在這次核准的清單裡**）：`libboost-thread-dev`、`libpcap-dev`、`libpcl-dev`、`ros-jazzy-pcl-conversions`。在 Jetson 上模擬安裝（`apt-get -s`）：升級 25 個、新裝 312 個、**移除 0 個**、沒有 nvidia / cuda / linux- 開頭的套件、沒有 OpenCV；下載約 330 MB、安裝後約 1.4 GB（記錄在 Jetson `~/r550_prep_logs/apt_sim_lslidar_deps_20261013.txt`）。`libpcl-dev` 占大部分空間；如果只需要 LaserScan，之後可以考慮在 `~/vendor_ws` 裡把點雲部分拿掉，但這次沒有做。

## 6. 風險

1. **R550 的協定沒有官方文件**：目前的協定 module 只是依第三方程式寫的；C30D 控制板可能是不同的封包長度或欄位。
2. **阿克曼指令模式不明**：(v, ω) 或 (v, 轉向角) 選錯，車會轉錯方向或轉向量不對。
3. **STM32 的逾時停車不明**：如果 STM32 沒有自己的 watchdog，我們的 driver 必須保證送停止（和 WAVE ROVER driver 一樣）。
4. **M10P driver 的依賴很大**（PCL 約 1.4 GB）；microSD 目前剩 42 GB，空間夠，但安裝前要再確認。
5. **B2 的 `nav2-bringup` 會移除 OpenCV 4.6 的開發套件**（見 2026-10-13 R550 準備報告），要先解決再裝 Nav2。
6. WHEELTEC 原廠的 ROS 2 套件可能依賴 Humble 才有的套件或舊版 API；Jazzy 移植版有人做過，但不是我們的車型。

## 7. R550 到貨第一天要確認的事

不讓輪子在地面轉；需要通電測試時先架空，限速限時、旁邊有人。

1. 拍照記錄：整車、控制板（型號、韌體版本貼紙）、光達型號標籤、電池規格、接線。
2. 拿到 WHEELTEC 隨車的資料包（原始碼、協定文件、韌體版本），存到 Orin 的 `~/vendor_ws/`，**不要放進 repo**。
3. 控制板怎麼接主機：USB 序列（晶片是 CP2102 還是 CH343）、還是 40-pin UART、還是 CAN；Orin 上出現的裝置名稱（`ls /dev/ttyUSB* /dev/ttyCH343USB* /dev/ttyACM*`）。
4. 只讀測試協定：開序列埠，**不送任何指令**，只收 STM32 主動送出的封包，確認長度是不是 24 bytes、`0x7B` / `0x7D`、XOR 對得上（`wro2026_base_r550.protocol.FeedbackStream` 可以直接用），電壓欄位和電池實際電壓一致。
5. 從原廠資料確認：指令模式 (v, ω) 或 (v, 轉向角)、byte 1 / 2 的意義、STM32 的指令逾時。
6. 光達：接在 USB 序列還是網路；`lidar_model: M10P` 用官方 driver 讀到 `/scan`（需要先裝第 5 節的套件）。
7. 量測：軸距、前後輪距、輪徑、最大轉向角、光達 / IMU 相對 `base_link` 的位置，寫進 URDF。
8. 供電：電池電壓與容量、Orin 從哪裡取電（R550 原本是給 Raspberry Pi 5）、急停方式。
9. 確認完協定後，才寫 `wro2026_base_r550` 的 ROS node（沿用 `/cmd_vel` 逾時、限速、回授遺失、低電壓），先架空驗證 stop / timeout。
10. 之後接 `wro2026_control` 時，`race_node` 一律加 `start:=false` 啟動（它的 `start` 預設是 true，有 `/scan` 和 `/odom` 就會開始動），見 [INTERFACES.md](INTERFACES.md) 第 3 節。
