# wro2026_real

實車的底盤以上層：bringup、機器人描述（URDF）、參數化的 line detector。
不依賴 Gazebo / ros_gz / turtlebot3，也不依賴 WAVE ROVER 的差速專屬行為；底盤只透過 `wro2026_base_waverover` 的 topic 溝通。

## 內容

| 檔案 | 用途 |
|---|---|
| `launch/bringup.launch.py` | 底盤 driver（include `wro2026_base_waverover`）+ robot_state_publisher。`transport`、`host`、`port`、`max_speed`、`cmd_timeout`、`publish_odom_tf`、`config` 轉給底盤 |
| `launch/line_detector.launch.py` | 只啟動 line detector（需要相機 driver，目前沒有） |
| `urdf/wave_rover.urdf` | 最小模型：`base_link`、`imu_link`、`camera_link`（placeholder）。尺寸全部 UNKNOWN，是暫定值 |
| `config/line_detector.yaml` | HSV 閾值、面積、ROI、影像 topic |
| `wro2026_real/line_detection.py` | 純函式「影像 → 偵測結果」，只用 numpy |
| `wro2026_real/line_detector_node.py` | ROS 節點，只做接線 |

**這裡沒有任何會讓車自主行駛的 launch**（CLAUDE.md：stop 與 timeout 在架空狀態驗證前不寫）。bringup 不發佈 `/cmd_vel`。

```bash
ros2 launch wro2026_real bringup.launch.py                                   # UART
ros2 launch wro2026_real bringup.launch.py transport:=http host:=192.168.0.16
```

TF：`odom → base_link`（底盤 driver，低品質 dead reckoning）→ `imu_link`、`camera_link`（robot_state_publisher，static）。

## `/line_detection` 格式（脆弱，之後要改）

`std_msgs/Float32MultiArray`，9 個值，和模擬版 `src/wro2026_sim/scripts/line_detector.py` 相同：

| index | 意義 |
|---|---|
| 0 / 4 | 橘 / 藍 是否偵測到（1 / 0） |
| 1 / 5 | 重心 x（整張影像的像素座標），沒偵測到是 -1 |
| 2 / 6 | 最大色塊面積（像素） |
| 3 / 7 | 面積 ≥ `trigger_area`（1 / 0） |
| 8 | 方向：1 = 順時針（藍線在左半）、-1 = 逆時針、0 = 未知 |

**這個格式只靠位置表示意義，很脆弱**：多一個欄位或換順序，訂閱端不會報錯而是讀錯值。之後應該改成自訂 message（例如 `wro2026_msgs/LineDetection`），這次先維持相容。

和模擬版的差異：面積是最大 8-連通色塊的像素數（模擬版用 OpenCV `contourArea`，對 w×h 矩形是 (w-1)(h-1)，略小）；cx 是像素重心；ROI 可以裁上下左右，cx 仍是整張影像座標。測試裡有和 OpenCV 版演算法的對照（有 OpenCV 的機器才跑）。

## 依賴

- 核心 `line_detection.py`：只要 numpy（Jetson 有 1.26.4）。
- 節點的影像轉換：rgb8 / bgr8 / rgba8 / bgra8 直接用 numpy 轉，不需要 cv_bridge。**其他格式（yuv422、mono8、compressed）需要 cv_bridge，Jetson 目前沒裝**，收到會略過並警告。
- 相機 driver：型號 UNKNOWN，Jetson 沒有任何相機 package，也沒有 `/dev/video*`。
- robot_state_publisher：Jetson 有裝；沒裝時 bringup 會略過並提示。

## 測試

```bash
cd src/wro2026_real && python3 -m pytest      # 只需要 numpy + pytest
colcon test --packages-select wro2026_real && colcon test-result --verbose
```

用 numpy 合成影像測：只有橘線、只有藍線、沒有線、兩條都有，另外有面積門檻、trigger、ROI、參數化 HSV、影像格式轉換、URDF 結構。
