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

**2026-11-08 量測（Orin 裝上車、線整理好後）：`'168:178,195:213,228:243'`。** 車子靜止，`range_min` 暫時放寬到 0.01 m 錄 30 秒（303 圈），在 4 個地點錄了 5 次。174° 和 196.5° 每次都在；線的部分中途整理過兩次，以最後一次為準（地點不同時跟著車子走的才算是車子自己）：

| 角度 | 擋到的東西 | 距離 |
|---|---|---|
| 168–178° | 正後方：Orin 底下 C30D 上的東西 | 0.12–0.16 m |
| 195–213° | 右後：銅柱（196.5°）、線（206–212°，多留一格：212.25° 偶爾有線的邊緣） | 0.16–0.19 m |
| 228–243° | 右側：Orin USB 那一側的線（237–239° 0.12 m，其他角度太近量不到） | 0.12 m |

共遮 44°，其中 27° 原本就被擋住、沒有回波，真正少掉的視野約 16°。這些點都比 `range_min` 0.1 m 遠，不遮的話 Nav2 / slam_toolbox 會以為車子旁邊永遠有障礙物。**線的位置一動，遮罩就要重量**（線離光達越近擋越多：同一束線在 11 cm 處擋 42°，在 17 cm 處擋 8°）。黑色霧面的東西（例如垃圾桶）光達完全看不到。資料在 Windows `下載\r550_check\scanmask_20261108_*`。
