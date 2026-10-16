# wro2026_base_r550

> **協定來源：WHEELTEC 官方文件**「串口通信控制与反馈」V1.0（2024-06-04）與「阿克曼转向角度指令计算」V1.0（2021-10-26），在 R550 資料包的 `1.WHEELTEC ROS机器人通用资料/2.运动底盘的控制与运动学解析/5.串口控制讲解/`。文件裡印出的範例封包都寫成測試，全部通過。**還沒在實車上錄封包驗證**。（最早是依兩份第三方 driver 寫的：[robotverseny/drivers](https://github.com/robotverseny/drivers) `26a63c5`、[shupx/akm_driver](https://github.com/shupx/akm_driver) `075e0f1`。）

WHEELTEC R550 阿克曼版（比賽車）底盤的協定 module。這次只有純 Python 的編碼 / 解碼，**沒有任何會送指令的 ROS node**；等 R550 到貨、確認協定之後再寫 driver，並沿用 `wro2026_base_waverover` 的安全機制（`/cmd_vel` 逾時、限速、回授遺失、低電壓）。

## 內容

| 檔案 | 用途 |
|---|---|
| `wro2026_base_r550/protocol.py` | 指令封包編碼（Twist 模式、Ackermann 模式）、回授封包解碼、XOR checksum、單位換算、從序列位元組流重組封包 |
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
- **STM32 自己有沒有指令逾時停車**：官方文件沒寫，driver 一定要自己做逾時停車。

完整調查見 `docs/real/R550_COMPATIBILITY.md`，研讀筆記見 `docs/research/R550_STUDY_NOTES.md`。

## 測試

```bash
cd src/wro2026_base_r550 && python3 -m pytest
colcon test --packages-select wro2026_base_r550 && colcon test-result --verbose
```
