# 尚待確認事項

## 已知
- 使用者現在手邊有一台實體車與 Jetson Orin。
- 目標是開始實車開發。
- 歷史 repo：`/home/ray/ros2_racecar`。
- 歷史禁止寫入：`/home/ray/ros2_ws`。
- 舊模擬 controller SHA：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`。
- Phase 7E-AE 沒有 calibration candidate。
- Phase 7G 最終為 `PROVEN_RESIDUAL_ODOM_ARCHITECTURE_BLOCKER`；舊 runtime slots 不可重跑。

## 必須補齊
1. 車體是 differential、Ackermann 還是其他？
2. 車體品牌/型號/說明書。
3. 馬達、encoder、driver、MCU 型號。
4. Jetson Orin Nano/NX/AGX？RAM？carrier board？
5. JetPack/L4T、Ubuntu、ROS 2 狀態。
6. LiDAR、相機、IMU 型號與到貨狀況。
7. 電池、DC-DC、電源分支、急停。
8. 輪徑、輪距/軸距、最大轉向角。
9. 程式實際在哪台機器/哪個 repo；是否有 remote。
10. 實車程式要獨立 repo/branch，還是只把既有 repo 當 reference？

## 決策規則
不清楚就標 UNKNOWN；不預設車型；不先升級 JetPack/ROS；不在確認電氣規格與 stop path 前驅動馬達；不在確認 Git/備份前刪除檔案。
