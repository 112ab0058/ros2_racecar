# C70 相機校正

更新：2026-11-12。相機：WHEELTEC C70（USB 名稱 Integrated Webcam，0bda:3031），Orin 上是 `/dev/video0`。

## 結果

| 項目 | 值 | 來源 |
|---|---|---|
| 解析度 | 1280×720 MJPG，約 28 fps | Orin 實測 |
| 內參 | `c70_ost_1280x720.yaml`（ROS camera_info 格式） | 23 張拍、用 15 張，重投影誤差 RMS 0.47 px |
| fx / fy / cx / cy | 822.8 / 819.2 / 584.2 / 400.3 px | 同上 |
| 視角 | 水平 75.8°、垂直 47.4° | 由 fx、fy 算 |
| 畸變 k1 k2 p1 p2 k3 | −0.424 +0.195 −0.0002 +0.0045 −0.048 | 桶狀變形明顯，見 `c70_undistort_check.jpg`（左原圖、右拉直） |
| 外參 | `c70_extrinsics.yaml` | 棋盤格平放地上，2 張 |
| 離地高度 | 0.103 m | Ray 用尺量「10 cm 多一點」，一致 |
| 俯角 / 側傾 / 偏航 | −0.5°（微微朝上）/ −0.6° / +1.4° | |
| 相機在 base_link 前方多遠 | 0.20 m | Ray 用尺量：鏡頭在保險桿最前端後方 4 cm；保險桿在 base_link 前方 0.24 m |

相機看得到的地板：畫面最下緣中間是鏡頭前 0.25 m，最下緣兩角是前方 0.17–0.20 m、左右各 0.16–0.19 m。牆底找錯 1 個像素時，距離誤差在 0.25 m 約 1 mm、0.54 m 約 4 mm、0.91 m 約 1 cm。

## 怎麼做的（重做時照這個）

1. 印 `checkerboard_9x6_25mm_A4.pdf`，選「實際大小 / 100%」，量比例尺要剛好 10 cm，貼在平的硬板上。
2. 內參（Orin，只開相機）：`python3 ~/racecar_ws/tools/cam_calib_capture.py intrinsic`，拿著板子在相機前 30–80 cm 換位置、距離、傾斜，收 25 張。
3. 計算：`python3 tools/real/calibrate_camera.py intrinsic <照片資料夾>`。
4. 外參：棋盤格紙平貼在車頭正前方地上（不要留在盒子上，盒子有厚度），最近的格子離鏡頭約 40 cm；`cam_calib_capture.py floor` 拍 3 張，在電腦上跑 `calibrate_camera.py floor <資料夾> --intrinsics <ost.yaml> --front-to-row <保險桿到最近一排交叉點的距離>`。

## 踩過的坑

- **延長線**：前兩條延長線不支援 USB 2.0 高速。第 1 條在 Orin 上只跑 12 Mbps（只給 160×120），換孔後一開始傳影像就失敗（STREAMON 錯誤 EPROTO）；第 2 條接電腦連裝置描述都讀不到。換第 3 條後 Orin 和電腦都正常。換線要選標示 USB 2.0 High-Speed / 480 Mbps 的。
- **Orin 的 OpenCV 4.6**：用路徑 `/dev/video0` 開相機拿不到畫面，要用編號 `0`；棋盤格平放在地上（畫面裡很扁）時找不到角點，電腦上的 OpenCV 4.13 找得到，所以外參在電腦上算。
- 移動中拍到的照片會糊，誤差 2–4 px，計算時會自動丟掉；拍照程式已改成板子停住才收。
