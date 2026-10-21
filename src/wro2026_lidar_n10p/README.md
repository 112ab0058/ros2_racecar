# wro2026_lidar_n10p

LSLIDAR N10P 光達的 ROS 2 driver（純 Python），**只讀**：不會送任何東西給光達（N10P 通電後自己就會一直送資料）。

> **協定來源**：LSLIDAR 官方 ROS 2 driver，分支 `M10P/N10P`（[Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver) commit `8bb760c`，2023-05-29）的 `lidar_name == "N10_P"` 部分。2026-10-17 在我們的光達上驗證通過。

為什麼不用官方 driver：官方的是 C++，只在 Foxy / Humble 測過，要另外安裝 Boost、PCL、libpcap、pcl_conversions（約 1.4 GB）。我們只需要 `/scan`，所以照它的協定寫成 Python，和 `wro2026_base_r550` 同一套做法。

## 協定摘要

460800 baud。每包 108 bytes：`A5 5A` 開頭；byte 5–6 起始角度（度 × 100）；byte 7–102 是 16 個點，每點 6 bytes（距離 mm 2 bytes、強度 1 byte，再一組第二回波）；byte 105–106 結束角度；最後 1 byte 是前 107 bytes 加總取低 8 位元。距離 `0xFFFF` = 沒有回波。

光達的角度是**順時針**，ROS 是**逆時針**，所以 ROS 角度 = 360° − 光達角度。

## 內容

| 檔案 | 用途 |
|---|---|
| `wro2026_lidar_n10p/protocol.py` | 封包解碼、從位元組流重組、把一圈的點分到固定數量的 bin |
| `wro2026_lidar_n10p/listen.py` | 命令列工具（不需要 ROS）：每秒印封包數、每秒圈數、壞封包數、最近和最遠的距離；`--raw-out` 存原始位元組 |
| `wro2026_lidar_n10p/scan_node.py` | ROS 2 node `n10p_scan`：發佈 `/scan`（`sensor_msgs/LaserScan`）、`/diagnostics` |
| `config/n10p.yaml`、`launch/scan.launch.py` | 參數和 launch |
| `test/test_protocol.py` | 用合成封包測解碼、重組、角度方向、只讀保證、參數和 yaml 一致 |

## 第一次接上（明天有延長線之後）

1. 把光達的 USB 線接到 Orin，光達會開始轉。
2. 找裝置：`ls -l /dev/serial/by-id/`、`lsusb`
3. 只聽：`ros2 run wro2026_lidar_n10p listen --port /dev/serial/by-id/<光達> --seconds 10 --raw-out ~/n10p_first.bin`
   - 預期：每秒約 300–340 包（5400 點 ÷ 16）、每秒 6–12 圈、`bad=0`、最近和最遠的距離合理
4. 啟動 node：`ros2 launch wro2026_lidar_n10p scan.launch.py port:=/dev/serial/by-id/<光達>`，然後 `ros2 topic hz /scan`

## 要在實車確認的

- 封包 byte 2–4、103–104 的意義（官方 driver 沒用到）
- ~~光達的 0° 朝哪個方向~~：2026-10-17 實測**光達 0° 正對車頭**（車頭前放一本立著的書，出現在 350°–6°），`angle_offset_deg = 0`；人站在車左後方，出現在 ROS 座標的左後方 → **順時針轉逆時針的換算正確，左右沒有鏡像**
- ~~一圈實際有幾個點~~：2026-10-17 實測每秒 10 圈、約 533 點／圈（0.68°），`bins` 從 720 改成 480（0.75°）
- 支架擋住的角度：在 `/scan` 裡看正後方有沒有很近的點

## 測試

```bash
cd src/wro2026_lidar_n10p && python3 -m pytest
```

## 遮掉自己的車（`mask_sectors_deg`）

Orin 裝在光達後上方，會擋到光達正後方一段角度。在 `config/n10p.yaml` 設 `mask_sectors_deg: '150:210'` 這種格式（ROS 角度：0° = 車頭、逆時針為正，可以跨過 360°，例如 `350:10`；多段用逗號分開），範圍內的光束一律送 inf（沒有回波），slam_toolbox 和 Nav2 就不會把自己的車當成障礙物。角度要等 Orin 裝好後，看 `/scan` 裡離光達很近（< 0.3 m）的點在哪些角度再決定。
