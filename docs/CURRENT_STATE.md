# WRO 2026 — 當前狀態快照
更新時間：2026-05-19

---

## 硬體環境
| 項目 | 細節 |
|------|------|
| 實車運算板 | NVIDIA Orin Nano 8GB，Ubuntu 22.04 + JetPack 6.2 |
| 遠端存取 | SSH + VNC，Tailscale IP：100.79.149.3 |
| 模擬環境 | WSL2 Ubuntu 24.04，ROS2 Jazzy，Gazebo Harmonic v8 |
| 實車環境 | Orin Nano，ROS2 Humble |
| 感測器 | 深度相機（RGB-D）+ 光達（LiDAR） |

---

## 現在做到哪

**Step 1~5 全部完成 ✅**
**下一步：Step 6 視覺 PID 直走控制**

### 已完成清單
- launch 一鍵啟動，Gazebo `-r` 自動播放 ✅
- ground_truth_odom 節點，不漂移 ✅
- TF chain 通，LiDAR 掃得到牆壁 ✅
- 地圖 66×66 存出，品質良好 ✅
- 相機換成 `turtlebot3_burger_cam`，`/camera/image_raw` 5.5Hz ✅
- 橘/藍線位置、角度、顏色全部修正完成 ✅
- 三個世界檔：open_wide / open_narrow / obstacle_wide ✅
- DDS/CycloneDDS 網路設定穩定 ✅
- GitHub repo + README 上傳 ✅
- line_detector.py：橘/藍線 HSV 偵測 + 觸發門檻 ✅

### Step 5 驗證結果
- 橘線：area=2190~3479，TRIG 正常 ✅
- 藍線：area=300~500，TRIG 正常 ✅
- 方向判斷：dir=1.0/−1.0 ✅
- 面積門檻 200，觸發門檻 1000
- HSV 橘色：H(8-30) S(150-255) V(150-255)
- HSV 藍色：H(100-130) S(150-255) V(50-255)

---

## /line_detection topic 格式
```
data[0] orange_detected   1=有 0=無
data[1] orange_cx         重心x像素 無則-1
data[2] orange_area       面積
data[3] orange_trigger    1=超過觸發門檻
data[4] blue_detected     1=有 0=無
data[5] blue_cx           重心x像素 無則-1
data[6] blue_area         面積
data[7] blue_trigger      1=超過觸發門檻
data[8] direction         1=順時針 -1=逆時針 0=未知
```

---

## 完整實作規劃

| Step | 項目 | 狀態 |
|------|------|------|
| 5 | 橘/藍線 HSV 偵測 | ✅ |
| 6 | 視覺 PID 直走控制 | 🔄 進行中 |
| 7 | YOLO26n 紅/綠柱偵測 | ⏳ |
| 7b | obstacle 多種柱子配置 SDF | ⏳ Step 7 時做 |
| 8 | 狀態機整合 | ⏳ |
| 8b | launch 多起始位置參數 | ⏳ Step 8 時做 |
| 9 | 停車偵測與執行 | ⏳ |
| 9b | 停車位 SDF 修正 | ⏳ Step 9 前做 |
| 10 | 移植 Orin 實車 | ⏳ |

---

## 轉彎邏輯設計
- 順時針：orange_trigger=True → 右轉
- 逆時針：blue_trigger=True → 左轉
- 行駛方向由啟動時藍線位置判斷（Step 8 實作）
- 轉彎中不再看線，由狀態機控制（Step 8 實作）

---

## GitHub
- Repo：https://github.com/112ab0058/ros2_racecar
- Commit 1：2026-05-18（Step 1-4）
- Commit 2：2026-05-19（Step 5 line_detector）
- 第三個 commit 截止：6 月中
- 最終截止：7/17

---

## 關鍵檔案
| 檔案 | 路徑 |
|------|------|
| Launch | `~/ros2_ws/src/wro2026_sim/launch/wro2026_sim.launch.py` |
| line_detector | `~/ros2_ws/src/wro2026_sim/scripts/line_detector.py` |
| 世界檔 | `~/ros2_ws/src/wro2026_sim/worlds/` |
| DDS 腳本 | `~/ros2_ws/ros2_wsl_network.bash` |
| GitHub | `~/ros2_racecar/` |

---

## 啟動方式
```bash
# 模擬
ros2_wsl_fix_loopback && ros2 daemon start
ros2 launch ~/ros2_ws/src/wro2026_sim/launch/wro2026_sim.launch.py

# line_detector
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/wro2026_sim/scripts/line_detector.py
```

---

## 不要動的東西
| 項目 | 原因 |
|------|------|
| SDF 牆壁高度 0.3m | 改回 0.1m LiDAR 掃不到 |
| 車子起始 (0, 0.9, 0.01) | 在外圈走道 |
| 橘/藍線位置角度 | 已精確對到內外牆 |
| DDS/CycloneDDS 設定 | 已驗證穩定 |
| ground_truth_odom | 不能移除 |
| line_detector HSV 範圍 | 模擬已調好，實車另外調 |

---

## 待確認
| 項目 | 狀態 |
|------|------|
| 實車相機型號 | ❓ 影響 HSV 重調 |
| 實車車長 | ❓ 影響停車位 SDF |
| obstacle_wide 起始距停車限制塊過近 | ⚠️ Step 9 前修 |
