# wro2026_lidar_n10p

LSLIDAR N10P 光達的 ROS 2 driver（純 Python），**只讀**：不會送任何東西給光達（N10P 通電後自己就會一直送資料）。

> **協定來源**：LSLIDAR 官方 ROS 2 driver，分支 `M10P/N10P`（[Lslidar_ROS2_driver](https://github.com/Lslidar/Lslidar_ROS2_driver) commit `8bb760c`，2023-05-29）的 `lidar_name == "N10_P"` 部分。**還沒在我們的光達上驗證**（2026-10-16 寫好，等 USB 延長線）。

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
- 光達的 0° 朝哪個方向（車頭？），不對就改 `angle_offset_deg`
- 一圈實際有幾個點，`bins` 要不要調整（720 = 0.5°）
- 支架擋住的角度：在 `/scan` 裡看正後方有沒有很近的點

## 測試

```bash
cd src/wro2026_lidar_n10p && python3 -m pytest
```
