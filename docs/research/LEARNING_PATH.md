# R550 實車學習路線

> **2026-10-16**：時程改成 4 週版，以 [PROJECT_PLAN.md](PROJECT_PLAN.md) 為準。下面的觀念和小練習仍然適用；第 5 階段改成 slam_toolbox + Nav2 + AI 任務層。

一邊把車做起來、一邊學。每個階段包含四部分：**要做的事**、**要懂的觀念**、**小練習**（用自己的話或程式驗證）、**要記錄的數據**（寫進 [LOGBOOK.md](LOGBOOK.md)，之後是專題報告的素材）。

順序依照「先看、再動、最後自己跑」。前一階段沒完成，不要跳到下一階段。

**全程安全原則**：C30D 控制板上的 **Motor 開關**，在第 1、2 階段都放 **OFF**。第一次讓輪子轉一定要先架空、限速、限時，旁邊有人，手放在開關旁。

---

## 第 1 階段：認識這台車（到貨第一週，輪子不轉）

**要做的事**
1. 充電（12.6 V 充電器接「充電口」）
2. **掃車上標籤的 QR code，下載產品資料**。這是最重要的一步，原始碼、STM32 通訊協定、C30D 說明都在裡面
3. Motor 放 OFF，單獨開車子，記下 OLED 顯示的內容
4. 把車上的 USB 線（STM32、光達、相機）接到 Orin，**只做辨識，不送任何資料**：`lsusb`、`ls -l /dev/serial/by-id/`、`sudo dmesg | tail`、`v4l2-ctl --list-devices`
5. **只「聽」STM32**：用序列埠工具接收 STM32 主動送出的封包，不送任何東西。拿來和 `wro2026_base_r550` 的協定解讀比對
6. 量尺寸：軸距、前後輪距、輪徑，記下最大轉向角

**要懂的觀念**
- **ROS 2 的基本單位**：node（程式）、topic（資料通道）、message（資料格式）、launch（一次啟動多個 node）、parameter（不改程式就能調的設定）
- **序列通訊**：baud rate（每秒傳幾個位元，兩邊要一樣）、封包格式（開頭 / 結尾的固定位元組 + 資料 + 檢查碼）、XOR checksum（把所有位元組互相 XOR，用來檢查傳輸有沒有出錯）
- **USB 裝置在 Linux 上的樣子**：序列裝置是 `/dev/ttyUSB*` 或 `/dev/ttyACM*`，相機是 `/dev/video*`。`/dev/serial/by-id/` 的名稱不會因為插拔順序而改變

**小練習**
- 從錄到的 STM32 封包，**用手算出一筆的電池電壓**，和 OLED 顯示的比對
- 用自己的話解釋：為什麼第一步只聽、不送？

**要記錄的數據**：OLED 內容、各 USB 裝置的 ID、錄到的封包樣本（存檔）、量到的尺寸

---

## 第 2 階段：讓 ROS 2 看到車子（輪子還是不轉）

**要做的事**
1. 寫（或改用廠商的）底盤 driver：發佈 `/odom`、`/imu/data`、`/battery`，**先不接收 `/cmd_vel`**
2. 光達 driver → `/scan`；相機 driver → `/image_raw`
3. URDF 填入真實尺寸，用 TF 把 `base_link`、`laser`、`camera_link`、`imu_link` 串起來
4. 用 Foxglove 在電腦上同時看光達點雲、相機畫面、TF

**要懂的觀念**
- **REP-103（單位與方向）**：長度用公尺、角度用弧度；x 往前、y 往左、z 往上；逆時針為正
- **REP-105（座標系）**：`map`（全域地圖）→ `odom`（連續但會漂移）→ `base_link`（車身）。為什麼要分 `map` 和 `odom` 兩層
- **TF**：每個感測器裝在車上不同位置，TF 負責換算「光達看到的點」在車身座標的位置
- **LaserScan 訊息**：`angle_min`、`angle_increment`、`ranges[]` 的意思

**小練習**
- 推車子（Motor OFF，用手推）往前 1 公尺，看 `/odom` 增加多少
- 用 `ros2 run tf2_tools view_frames` 畫出 TF 樹

**要記錄的數據**：手推 1 m 時 odom 讀到的距離（推 3 次）、IMU 靜止 1 分鐘的 yaw 漂移量、光達 / 相機的實際頻率

---

## 第 3 階段：讓車子動（架空 → 地面手動遙控）

**要做的事**
1. 底盤 driver 開始接收 `/cmd_vel`，加上和 WAVE ROVER 一樣的安全機制：逾時停車、限速、回授遺失停車、低電壓拒絕
2. **架空測試**：Motor ON，低速，確認前進、後退、左右轉向方向正確，停車保護有效
3. 地面手動遙控：teleop，最低速，在空地慢慢開

**要懂的觀念**
- **阿克曼轉向（腳踏車模型）**：前輪轉 δ 角時，車子繞一個圓心轉彎。角速度 ω、速度 v、軸距 L 的關係是 `ω = v · tan(δ) / L`。所以**車子不能原地旋轉**，速度 0 時轉向沒有效果
- **最小轉彎半徑**：`R = L / tan(δ_max)`，決定場地轉角能不能一次轉過去
- **encoder → 距離**：`距離 = 2π × 輪半徑 × 脈衝數 / (每圈線數 × 減速比 × 倍頻)`。AB 相 encoder 用 4 倍頻時，500 線每圈會數到 2000 個脈衝

**小練習**
- 用量到的 L 和最大轉向角，**算出最小轉彎半徑**，再實際開一圈量地上的圓，比較誤差
- 確認 STM32 收到的是 (v, ω) 還是 (v, δ)：送固定的 v 和 ω，量實際轉彎半徑

**要記錄的數據**：停車延遲（ms）、直線行駛 2 m 時 odom 與捲尺的誤差、最小轉彎半徑的計算值和實測值

---

## 第 4 階段：感測器融合（專題的核心）

**要做的事**
1. 用 `robot_localization` 的 EKF，把輪子 odom 和 IMU 融合成一個更準的位置估計
2. 設計實驗：繞場地走固定路線（例如正方形 4 圈），比較「只用輪子」、「只用 IMU 算角度」、「融合」三種的誤差
3. 錄 rosbag，所有實驗都要能重播

**要懂的觀念**
- **為什麼要融合**：輪子會打滑（距離不準），IMU 的角度會慢慢漂移（陀螺儀偏差）。兩者的誤差特性不同，合在一起可以互補
- **卡爾曼濾波（EKF）的直覺**：預測（依運動模型推算下一刻在哪）→ 更新（用感測器量測修正），每個數據依「可信度」（covariance）加權
- **covariance 的意義**：數字越大代表越不相信這個感測器

**小練習**
- 故意把 IMU 的 covariance 調很大或很小，看融合結果怎麼變，解釋原因

**要記錄的數據**：三種方法回到起點時的位置誤差和角度誤差（多次平均）。**這組數據可以直接放進專題報告**

---

## 第 5 階段：自主行駛

**要做的事**
1. 實體場地（160 × 160 cm，見 `docs/sim/FIELD_PLAN_160CM.md`）：用 C70 拍場地上的橘線和藍線，調整 line detector 的 HSV 參數
2. 決策層（狀態機 / AI 任務層）先用 `start:=false` 之類的開關確認收到的資料都正確，再低速啟動
3. 之後才做 SLAM（`slam_toolbox`）建地圖、Nav2 導航

**要懂的觀念**
- **HSV 色彩空間**：為什麼辨識顏色用 HSV 比 RGB 穩定（亮度變化主要影響 V，不太影響 H）
- **狀態機**：控制器的 WAIT → FOLLOW → APPROACH → TURN 怎麼切換，每個狀態的離開條件
- **SLAM**：一邊建地圖、一邊在地圖上定位

**要記錄的數據**：每圈時間、成功率、失敗原因分類

---

## 參考資料

- [ROS 2 Jazzy 官方教學](https://docs.ros.org/en/jazzy/Tutorials.html)：先做 Beginner: CLI tools 和 Beginner: Client libraries
- [REP-103：單位與座標方向](https://www.ros.org/reps/rep-0103.html)
- [REP-105：移動機器人的座標系](https://www.ros.org/reps/rep-0105.html)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)：EKF 感測器融合
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 文件](https://docs.nav2.org)
- [Ackermann steering geometry（維基百科）](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)
