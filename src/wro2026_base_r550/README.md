# wro2026_base_r550

> **協定來自第三方程式 [robotverseny/drivers](https://github.com/robotverseny/drivers)（`turn_on_wheeltec_robot`，ROS 2 Jazzy，commit `26a63c5`）與 [shupx/akm_driver](https://github.com/shupx/akm_driver)（ROS 1，WHEELTEC 阿克曼車，commit `075e0f1`）；兩者都源自 WHEELTEC 原廠的 ROS driver，但不是 WHEELTEC 官方文件。未在實機驗證（UNVERIFIED）。**

WHEELTEC R550 阿克曼版（比賽車）底盤的協定 module。這次只有純 Python 的編碼 / 解碼，**沒有任何會送指令的 ROS node**；等 R550 到貨、確認協定之後再寫 driver，並沿用 `wro2026_base_waverover` 的安全機制（`/cmd_vel` 逾時、限速、回授遺失、低電壓）。

## 內容

| 檔案 | 用途 |
|---|---|
| `wro2026_base_r550/protocol.py` | 指令封包編碼（Twist 模式、Ackermann 模式）、回授封包解碼、XOR checksum、單位換算、從序列位元組流重組封包 |
| `test/test_protocol.py` | pytest；範例封包是**依來源程式的規則手算**的，不是從 R550 錄下來的 |

## 協定摘要（兩個來源一致的部分）

序列埠 115200 baud。

主機 → STM32，11 bytes：

| byte | 內容 |
|---|---|
| 0 | `0x7B` |
| 1, 2 | 保留（ROS 2 來源都送 0；ROS 1 阿克曼來源 byte 1 送 1，註解「產品型號」，byte 2 註解「使能」） |
| 3–4 | x 速度，int16 大端，mm/s |
| 5–6 | y 速度（只有全向底盤用） |
| 7–8 | Twist 模式：z 角速度 mrad/s；Ackermann 模式：轉向角 rad × 1000 / 2 |
| 9 | byte 0–8 的 XOR |
| 10 | `0x7D` |

STM32 → 主機，24 bytes：`0x7B`、stop flag、x/y/z 速度（mm/s、mrad/s）、加速度計 x/y/z（±2 g，16384 LSB/g）、陀螺儀 x/y/z（±500 deg/s，1 LSB = 0.00026644 rad/s）、電壓（mV）、byte 0–21 的 XOR、`0x7D`。多位元組都是大端。

## 未確認（到貨後要驗證）

- R550 的控制板（經銷商頁面寫 WheelBoard C30D）用的是不是這個 11 / 24 bytes 格式。網路上有用 WHEELTEC MiniBalance 控制板韌體的第三方專案，回授是 36 bytes（同樣 `0x7B` / XOR / `0x7D`），所以不同板子或韌體版本的長度可能不同。
- STM32 收的是 (v, ω) 還是 (v, 轉向角)；byte 1、2 要填什麼。
- 序列埠的裝置名稱（來源用 CP2102 `/dev/ttyUSB*` 或 CH343 `/dev/ttyCH343USB*`，再用 udev 建 `/dev/wheeltec_controller`）。
- STM32 自己有沒有指令逾時停車。
- 速度和 IMU 的正負方向、`base_link` 的定義。

完整調查見 `docs/real/R550_COMPATIBILITY.md`。

## 測試

```bash
cd src/wro2026_base_r550 && python3 -m pytest
colcon test --packages-select wro2026_base_r550 && colcon test-result --verbose
```
