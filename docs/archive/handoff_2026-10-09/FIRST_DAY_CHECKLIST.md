> 已封存（2026-10-13）。目前狀態見 docs/STATUS.md。

# 實車＋Jetson Orin：第一天 Checklist

## 安全
- [ ] 找到急停/馬達電源快速斷開。
- [ ] 車輪架空並固定車體。
- [ ] 馬達供電與 Jetson/MCU 供電是否分支，確認馬達啟停不會讓 Jetson 重啟。
- [ ] 確認電池電壓、極性、最大電流、保險絲。
- [ ] 確認 GPIO/UART/CAN/PWM 電平；不可把 Jetson GPIO 接 5V 或馬達/舵機訊號而不查手冊。
- [ ] stop command 未驗證前，不發出移動命令。

## 拍照
整車俯視/側面/底部；馬達/encoder；driver 端子；MCU；電池/DC-DC/保險絲；Jetson 型號與載板；LiDAR/相機/IMU 標籤；接線近照。

## 硬體表
| 項目 | 記錄 |
|---|---|
| Jetson | Orin Nano/NX/AGX、RAM、carrier board、JetPack/L4T |
| OS/ROS | Ubuntu、kernel、ROS 2 distro、RMW/DDS |
| 底盤 | differential/Ackermann/其他、驅動輪、轉向輪 |
| 馬達 | 型號、額定電壓/電流 |
| Encoder | 增量/絕對、CPR、x1/x2/x4、gear ratio、安裝位置 |
| Driver | 型號、介面、電壓/電流、保護機制 |
| MCU | 型號、韌體、USB/UART/CAN |
| LiDAR/Camera/IMU | 型號、介面、供電、driver |
| Power | 電池、DC-DC、Jetson/馬達供電、保險絲 |
| Safety | 急停、限流、watchdog、通信失效時行為 |

## Jetson 唯讀盤點命令（在 Jetson 執行，不要先升級）
```bash
date -Is
uname -a
cat /etc/os-release
cat /etc/nv_tegra_release 2>/dev/null || true
dpkg-query -W 'nvidia-jetpack' 'nvidia-l4t-core' 2>/dev/null || true
printenv | grep -E '^(ROS_DISTRO|RMW_IMPLEMENTATION|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH)='
lsusb
ip -brief address
df -h
free -h
```
ROS 2 已裝時：
```bash
ros2 doctor --report
ros2 pkg list | grep -E 'nav2|slam_toolbox|robot_localization|tf2|rplidar|ydlidar|orbbec'
```
命令不存在就記錄未安裝，不要為了讓它成功立刻安裝。

## 第一個測試
1. Jetson 穩定開機、電源不掉壓。
2. Jetson 能與 MCU/driver 通訊。
3. 車輪架空、低速、短時間命令可執行。
4. stop、timeout、communication loss 時馬達確實停止。
5. encoder 方向/計數正確。
6. 保存命令、輸出、版本、接線照片。
先不做 Nav2、不做整車自主行駛。
