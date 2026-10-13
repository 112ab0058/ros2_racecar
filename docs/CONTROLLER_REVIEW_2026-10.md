# 控制器檢討與重寫（2026-10）

範圍：`src/wro2026_sim/scripts/line_follower.py`（production SHA `de79e41c…`，4315 行）、Phase 7 odom campaign 的結果，以及取代它的新 package `src/wro2026_control`。

舊檔案與 `src/wro2026_sim` 全部沒有修改。

## 1. 結論

- 舊 controller 只能在**一個**情境下跑：Gazebo 真值座標、固定起點、逆時針、北側 (0, 0.42) 一顆黃色障礙物。約 40 個常數是場地絕對座標，換方向、換起點或換到實車都會失效。
- 避障的「正常完成」路徑在程式裡永遠到不了；每次避障都是撞進 RECOVER，再靠後面約 1000 行補丁收尾。
- 有數個會讓車永久停住的狀態（不會進 RECOVER_FAILED，也不會逾時）。
- Phase 7 的 odom 修正路線是建立在不存在的誤差上：模擬裡輪徑與輪距完全正確，所謂 residual 只是轉彎開始與結束時約 0.2 s 的暫態落差，0.5 s 內自行抵消。
- 新 controller 不用任何絕對座標，只用 yaw、行駛距離、LiDAR 與色線，所以同一份程式可以跑模擬也可以上實車。

## 2. 舊 controller 的問題（行號以 SHA `de79e41c` 為準）

| # | 問題 | 位置 | 影響 |
|---|---|---|---|
| 1 | `/odom` 是 Gazebo 真值（`race.launch.py` 啟動 `gazebo_ground_truth_odom.py`），決策大量使用絕對 x/y：速度區、避障區、相機 gate、中央島北牆 x 範圍、離開上方的目標點 (−0.08, 0.10) | 88–95、141–247、286–290 行 | 換起點、換方向（順/逆時針）、換實車全部失效 |
| 2 | 相機黃色 gate 只在 x∈[0.05, 0.25]、y∈[0.34, 0.45]、朝西時成立；LiDAR 觸發也要求 y > 0.28 | 286–290、907–916、2008–2011、2115–2125 行 | 只能處理北側那一顆障礙物 |
| 3 | `_choose_gap_direction` 算出要往哪邊閃，但 lane shift 的指令固定是倒車 −0.055 加左轉 +0.45；完成條件也只看正向 yaw 累積 | 112–113、2036–2043、3080 行 | 永遠往同一邊閃，跟空隙在哪無關；2041 行的「scan 鏡像」註解是在補符號問題 |
| 4 | `AVOID_PASS_ELAPSED_CAP_SEC = 0.5` 比 `AVOID_PASS_MIN_SEC = 1.0` 還短，且 0.5 s 檢查排在完成條件之前 | 122、128、3199–3212 行 | 正常完成與 3 s 逾時都到不了，每次避障都以 RECOVER（`elapsed_cap_500`）收場；RECOVER_EGRESS、POST_AVOID_LEAVE_TOP、clearance escape 等狀態都是在補這個 |
| 5a | RECOVER_EGRESS 失敗時只把 phase 設成 FAILED，state 不變，之後每個 tick 只送停止 | 1689–1700、1729–1731 行 | 永久停車，不會進 RECOVER_FAILED |
| 5b | clearance escape 進入 blocked 後每個 tick 都停車並 return，而且它在 POST_AVOID_LEAVE_TOP 的逾時檢查之前執行 | 2375–2378、2438–2439、3513–3518 行 | 逾時永遠不會被檢查到，永久停車 |
| 5c | top-exit retry 用完後每次呼叫都停車並 return True | 2527–2540 行 | 在 POST_AVOID_LEAVE_TOP 內永久停車；在 POST_TURN 內最多停 60 s |
| 6 | ALIGN 只要 0.45 s 就離開，完全不看角度誤差；後面的 `ALIGN_MAX_SEC` 判斷是死碼 | 2952–2960 行 | 轉彎角度沒對齊就進直線 |
| 7 | TURN 逾時直接當作轉完，把目標角當成新的直線方向 | 2918–2925 行 | 轉一半也會被當成轉完 |
| 8 | 收到第一筆 odom 之前，`_now()` 回傳 `time.monotonic()`，之後改用 sim time；狀態起點與牆面 PD 的時間基準混用 | 699–700、546、641、1608–1613 行 | 啟動時 PD 微分項暴衝 |
| 9 | 非 STRAIGHT 狀態一律忽略色線；沒有圈數計算，也不會停 | 1306 行 | 跑完 3 圈不會停；驗收只測到 1 圈 |
| 10 | 85 處 logger 呼叫，熱路徑每個 tick 組大量 f-string 與 JSON | 全檔 | 拖慢控制迴圈，log 難讀 |

## 3. Phase 7 odom campaign

完整證據在 `/home/ray/ros2_evidence/phase7g_residual_odom_campaign_20260807_062907/`。

- 7G 的最終失敗：Grouped-CV 最差 translation ratio 1.439 來自 R3 的 fold 0。該 fold 的 baseline 只有 0.28 mm/window，模型變成 0.40 mm，絕對差 0.12 mm。
- `TURN_HIGH_CURVATURE` 在凍結的速度上限下（0.05 m/s、0.30 rad/s）根本到不了。最緊的弧是曲率 12 /m，被歸類成 MEDIUM。7G-D-R 的 coverage gate 只檢查曲率 bins，7G-G 卻要求 regime 齊全，兩個 gate 互相矛盾。
- 根因：Generation A 的逐 window 資料顯示，穩態旋轉的 truth/nominal yaw 比值中位數是 1.0000000（p5–p95 在 ±3×10⁻⁶ 內），穩態直線的 residual 約 10⁻⁹。每次旋轉 truth 落後約 0.063 rad，接著的 STOP 內補回。整趟約 19 rad 的轉動，淨誤差 0.006 rad。
- 也就是說幾何沒有誤差可校正（7E-AE 必然「不可識別」）；residual 是轉換瞬間的暫態，只看當前 0.1 s window 的特徵在結構上學不到（7G 必然無法泛化）。
- 建議：封存，不再延續 odom 修正路線。定位應直接用 nominal odom 加上 IMU 與 LiDAR。

## 4. 新 controller：`src/wro2026_control`

| 檔案 | 內容 |
|---|---|
| `wro2026_control/core.py` | 控制邏輯，不依賴 ROS，可單獨測試 |
| `wro2026_control/race_node.py` | ROS 2 node：`/scan`、`/odom`（或 `/imu`）、`/line_detection` → `/cmd_vel`、`/race_status` |
| `wro2026_control/race_monitor.py` | 模擬驗收用：印出一行 RESULT 後結束 |
| `launch/sim_race.launch.py` | Gazebo 驗證（沿用 wro2026_sim 的 world、bridge、line_detector，不修改它們） |
| `tools/sim2d.py`、`tools/matrix.py` | 讀同一份 SDF 的離線 2D 模擬器與情境矩陣 |
| `test/test_core.py` | 單元測試與閉迴路測試 |

運作方式：

1. 只用 yaw 與行駛距離做短距離航位推算；車道中心線、車道方向（也順便消除 yaw 漂移）、前方牆位置每次掃描都從 LiDAR 重新量測。
2. 第一條看到的線決定方向（橘先＝順時針，藍先＝逆時針）；兩色同時出現不判斷。沒看到線時，用第一個彎「哪一側打開」決定或修正。
3. 參考路徑：車道中心線＋彎道四分之一圓（位置由量到的前方牆決定）＋下一條車道中心線。轉彎不需要獨立的狀態。
4. LiDAR 點（加上剛進入 LiDAR 盲區的近距離點）建成局部最近點地圖，可以算出任何姿態下車身矩形的精確間隙。
5. 規劃器：沿參考路徑每 5 cm 一站，每站選一個橫向偏移；以二階動態規劃同時檢查車身碰撞、曲率上限（可設成 Ackermann 的最小轉彎半徑）與偏離成本。障礙物、牆、彎道用同一套規則處理。
6. Pure pursuit 追路徑，再經一層短距離碰撞檢查。真的走不了時，沿剛走過的軌跡倒退，並記住卡住的位置，讓下次規劃換一條路。在同一條車道重複卡住 6 次就停車並回報 `blocked`，不會撞上去。
7. 依參考路徑計算彎數；3 圈後停在起點（用起點到前方牆的距離判斷，不受累積漂移影響）。
8. 前進指令一律 linear > 0，且遵守 `min_turn_radius`，不使用原地旋轉，換成轉向輪車也能用。

## 5. 驗證結果（離線 2D 模擬，同一份 `wro2026_field.sdf`）

通過條件：3 圈（12 個彎）、全程無碰撞、停在起點沿車道方向 8 cm 內。模擬含 LiDAR 雜訊 5 mm、scan 8 Hz、相機 10 Hz、控制 20 Hz。

| 測試組 | 內容 | 結果 |
|---|---|---|
| 固定情境 `matrix.py` | 無障礙、SDF 原配置、北側外側 1 顆、三條車道各 1 顆、四條車道各 1 顆、北側交錯 2 顆（間距 0.45 m）、北側內側 3 顆；順/逆時針各一次 | **14/14**，全部 0 次倒退，最小間隙 32–170 mm，停車誤差 ≤ 2.2 cm |
| 隨機配置 `matrix.py --random 12` | 每條車道 0–2 顆、只放在直線段、偏離中心 ≤ 12 cm、同車道間距 ≥ 0.40 m；12 組 × 2 方向 | **24/24**，倒退合計 1 次，最小間隙 ≥ 27 mm，停車誤差 ≤ 0.6 cm |
| 擾動 `matrix.py --robust` | yaw 漂移 0.003 rad/s、行駛距離 ±5 % 誤差、LiDAR 雜訊 12 mm、60 % 的線沒看到、Ackermann 最小轉彎半徑 0.25 m、起點前移 0.35 m、起始角偏 7° | **16/16** |
| 過窄配置 `matrix.py --stress` | 間距 0.28 m 的三顆交錯、斜向通道 0.24 m、障礙物在彎道轉彎線上 | 6/6 無碰撞；其中 2 組跑完 3 圈，4 組安全停車（`blocked`） |
| 單元測試 `pytest` | 方向判斷、不會撞前方牆、Ackermann 限制、scan 時間對齊、停止條件；以及經過 ROS 訊息轉換的 node 閉迴路 | 全部通過 |

過窄配置的說明：間距 0.28 m 的交錯，對最小轉彎半徑 0.12 m 的車在幾何上就過不去（用同一套規劃器在完美地圖下驗證過）；彎道轉彎線上的障礙物只留下約 1.5 cm 的可行帶。這類情況車會停下並回報原因，不會撞上去。

## 6. Gazebo 驗證（在 WSL）

```bash
# WSL
cd ~/ros2_racecar && git fetch origin && git switch control/pose-free-race-controller && git pull
rsync -a --delete ~/ros2_racecar/src/wro2026_control/ ~/ros2_ws/src/wro2026_control/
cd ~/ros2_ws && colcon build --packages-select wro2026_control wro2026_sim && source install/setup.bash
ros2 launch wro2026_control sim_race.launch.py                      # 逆時針、SDF 障礙物
ros2 launch wro2026_control sim_race.launch.py direction:=cw         # 順時針
ros2 launch wro2026_control sim_race.launch.py obstacles:="-0.30,0.40;0.15,0.60"
```

每次跑完會印出一行 `RESULT {...}`，然後整個 launch 自動結束。

## 7. 已知限制與下一步

- 驗證到目前為止只在離線 2D 模擬（含一個用假 ROS 模組跑 node 的測試）；Gazebo 的實際結果待跑。`obstacles:=` 參數靠 `gz service set_pose` 移動障礙物，靜態模型能否被移動也要在 Gazebo 確認。
- 輪子打滑、加減速動態在 2D 模擬裡沒有模型。
- 實車（WAVE ROVER）目前沒有光達也沒有 encoder：controller 需要 LiDAR，行駛距離要用 IMU 加指令積分或之後的 encoder 提供。
- 比狹窄情境（stress layouts）更擠的配置，車會安全停住（`blocked`），不保證跑完。
