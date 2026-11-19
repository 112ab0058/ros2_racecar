# wro2026_camera_c70

WHEELTEC C70 相機：校正檔，加上一個**只讀**的 node，用相機找出白地板上黑牆的牆腳，補光達的盲區。

## 為什麼需要

N10P 光達對場地的霧面黑牆，在 0.32 m 內完全沒有回波，0.32–0.45 m 只偶爾有（2026-11-13 實測，`docs/research/LOGBOOK.md`）。走道約 0.42 m 寬，兩側的牆剛好落在這個範圍。

11-13 第 4 次失敗的原因：在這段「時有時無」的距離只量到零星幾個點，程式把它們畫成一條假的牆，判斷「牆到這裡結束」，就提早轉彎。

相機看得到這些牆腳：地板很亮，牆很暗。

## 怎麼做（`wall.py`）

1. 灰階，5×5 模糊。每 4 欄一欄，從畫面最下面往上找，第一段連續 12 個比 `threshold`（110）暗的像素就是牆，那一列就是牆腳。最下面已經是暗的欄（牆比畫面下緣還近，或地板是暗的），或整欄都沒變暗的欄，都跳過。
2. 用內參去畸變，再用外參把像素的方向轉到 `base_link`，跟地板（離鏡頭 `height_m` 以下）求交點。
3. 換成「從光達看出去，每個光束的距離」（480 束，跟 `/scan` 一樣），發布 `camera/wall_scan`（`sensor_msgs/LaserScan`，frame `laser_link`）。

`scan_memory_node`（`wro2026_lidar_n10p`）把這些點放進記憶，位置用拍照當下的里程計姿態。光達量不到的光束，就用記憶裡的點補上。`/scan_filled` 的 intensity：0 是光達量到，1 是光達記憶，2 是相機記憶。

## 準確度和看得到的範圍

- 11-13 場地 10 組照片和同時錄的光達比對（光達已扣掉 3.6 cm 偏差）：相機比光達短 1.8 cm（中位數），平均誤差 4 cm。相機參數沒有改。
- 牆腳找錯 1 個像素，距離誤差在 0.25 m 約 1 mm、0.9 m 約 1 cm。所以只用 1.0 m 以內的點（`max_range`），更遠的交給光達。
- 地板看得到的範圍：鏡頭左右各約 45°（桶狀變形讓邊緣看得比較廣），距離鏡頭 0.26 m 以外（2026-11-20 用校正參數算）。**車子正旁邊看不到**，那裡要靠記憶。
- 在 11-13 的照片上，最近量到離光達 0.25–0.33 m 的牆。
- 每張照片處理約 4.5 ms（Windows）。

## 用法

只讀相機，不會讓馬達轉：

```bash
ros2 launch wro2026_camera_c70 wall.launch.py
ros2 topic hz /camera/wall_scan
```

`tools/real/orin_auto.sh` 會自動啟動它，並設 `image_rate:=5.0`。這樣 bag 裡每秒會存 5 張照片（`/camera/image/compressed`），之後可以用同一套 `wall.py` 重播。

參數在 `config/c70_wall.yaml`，跟 `wall_node.py` 的 `PARAMETERS` 必須一樣（有測試檢查）。

## 校正檔

- `config/c70_ost_1280x720.yaml`：內參（ROS camera_info 格式）
- `config/c70_extrinsics.yaml`：外參

這兩個檔案 2026-11-20 從 `docs/real/calib/` 搬到這裡，因為 node 在 Orin 上要讀得到。怎麼量出來的，見 `docs/real/calib/README.md`。重新校正後，把 `calibrate_camera.py` 寫出的檔案覆蓋這兩個。

## 注意

- 地板不是白的（例如家裡的木地板）時，大部分欄一開始就是暗的，會被跳過，相機就不給點。這不是壞掉。
- 地上的暗色線條或膠帶：在畫面上比 12 像素細的會被忽略（2 cm 寬的線，在 0.4 m 外約 10 像素）。更近、更粗的會被當成牆，但光達只要看到它後面還有牆，記憶就會把它清掉（`clear_margin`）。
- Orin 的 OpenCV 4.6 用 `/dev/video0` 加 `cv2.CAP_V4L2` 打開（跟 `cam_snap.py` 一樣，11-13 在場地可以用）。

## 測試

```bash
cd src/wro2026_camera_c70 && python3 -m pytest
```

其中一個測試會用真的校正參數（含畸變）畫一面 0.80 m 外的牆，再把它找回來，誤差要在 1 cm 內。
