# WRO ROS 2 Racecar 專案交接

## 1. 現在要做什麼
正式開始實車：使用者有實體車與 Jetson Orin。硬體型式、馬達/encoder/driver/MCU、Jetson 具體型號與 JetPack 尚未在本交接資料中確認。第一目標是辨識硬體與安全供電，再建立低階控制和 encoder odometry，最後才接 ROS 2 感測器、定位、Nav2 與 WRO 行為。

## 2. 模擬場地歷史基線
過去 WRO 風格模擬場地約 1.6 × 1.6 m；外牆內側約 ±0.785 m；中央島 nominal 0.400 × 0.400 m、collision 約 0.430 × 0.430 m；停車區 x=[0.525,0.785]、y=[-0.250,0.250]；右側走廊約 0.310 m；模擬車 footprint 約 0.140 × 0.178 m。這些只是模擬基線，實車與實際場地必須重新量測。

## 3. 過去模擬架構
- ROS 2 Jazzy / Nav2 曾在開發環境使用。
- TurtleBot3-style differential-drive simulation，`/cmd_vel` 為 `geometry_msgs/Twist`。
- 名義 wheel radius 0.033 m、wheel separation 0.160 m。
- 歷史正式接受 controller SHA：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`。
- 歷史禁止寫入 workspace：`/home/ray/ros2_ws`。
- 模擬地圖曾在 `/home/ray/ros2_racecar/src/wro2026_sim/maps/phase7_nav2_field_map.yaml` 與同名 `.pgm`。

這個 controller SHA 只用於識別舊模擬 artifact，不代表它可直接驅動實車。不要把模擬 package 原封不動部署到 Jetson。

## 4. 歷史工作摘要
### 早期 Phase 0–6
重點在 Gazebo controller、避障/導航、reset lifecycle、trace/evidence correctness、production acceptance 與 rollback。重要習慣：固定 artifact identity、formal/workspace/installed SHA/cmp、runtime slot 不重跑、每 phase 保存 evidence/manifest、temp hooks/package 結束後清除、不覆寫使用者既有 Git 變更。

### Phase 7 導航/場地
- 7B/7B-R：模擬場地基線通過。
- 7C/7C-R：TF/sensor/odom baseline；曾發現 IMU 0 messages、depth 缺失、odom covariance 全零等問題。
- 7D/7D-R：SLAM mapping 通過；地圖路徑如上。
- Localization gates 曾包含 coverage ≥95%、position error ≤0.15 m 比例 ≥95%、yaw error ≤0.3 rad 比例 ≥95%、max position ≤0.25 m、max yaw ≤0.5 rad；這些是歷史模擬 gate，不自動適用於實車。

### Wheel geometry calibration
- 模擬名義 rL=rR=0.033 m、B=0.160 m。
- `/odom` 由 Gazebo DiffDrive plugin 依 wheel joint position 內部積分，不是 world truth pose。
- joint names：`wheel_left_joint`, `wheel_right_joint`；單位 rad/rad·s⁻¹。
- Dataset：`/home/ray/ros2_evidence/phase7eady_measurement_repaired_dataset_20260807_004700/measurement_repaired_local_wheel_odom_calibration_dataset.yaml`
- Dataset SHA：`2e650de7f3667fa2d335b14ffc44134e0ec9aa6b667721f36490577cb306f94e`
- Phase 7E-AE 分類 `7EAE2 WHEEL_GEOMETRY_CALIBRATION_NOT_IDENTIFIABLE_OR_NOT_GENERALIZABLE`。最大參數相關約 0.9996786（gate <0.995），bootstrap 成功率 82.5%（gate ≥90%）；沒有 candidate/freeze/covariance。
- AE-R 判斷 nominal geometry 在 frozen holdout gates 不足，但 excitation 欠識別，不能斷言 constant geometry model 本身失效。
- AE-S/S-R/S2/S3 在 frozen constraints 下找不到安全且可識別的高資訊 wheel excitation。Body Twist 與 differential wheel velocity 是可逆映射；direct-wheel API 沒增加運動基底，frozen plugin 也沒有該介面。

### Residual odometry
Campaign 路徑：`/home/ray/ros2_evidence/phase7g_residual_odom_campaign_20260807_062907/`
- 最終分類：`PROVEN_RESIDUAL_ODOM_ARCHITECTURE_BLOCKER`
- 停止原因：`STOP_RUNTIME_BUDGET_EXHAUSTED`
- Qualification、Generation B training、holdout 各一次有效 runtime，無重跑。
- Dataset/isolation gates PASS；frozen one-SE rule 選 R3、λ=0.0001。
- Grouped-CV 最差 translation ratio 1.439396 > 1.10，且缺少 `TURN_HIGH_CURVATURE` coverage。
- Runtime slots 已用盡；不得重跑、改 folds、放寬 gates 或事後改選 R1/R2。
- 未建立/提升 residual candidate；holdout model metrics 未開封。
- 未進 residual implementation、AMCL Level 1 或 Phase 7F。
- Campaign manifest 562/562 PASS；SHA `0de44b3359f618b4e94b7077f62fbd95750767e296765943465fced4f697793a`。
- Production controller SHA 保持 `de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`；formal/workspace/installed cmp=0。Temp artifacts 已清除，ROS/Gazebo processes 0，`/home/ray/ros2_ws` 未寫入。

## 5. 如何解讀負結果
這些負結果只證明在既有 frozen 模擬模型、特定 excitation、固定 gates 和有限 runtime budget 下，特定 wheel calibration/residual odom 路線未產生可接受候選。不代表 WRO 實車、Nav2 或所有 odometry 方法不可行。實車有不同硬體與量測條件，應先從真實硬體識別與低階控制開始。

## 6. 不可破壞的規則
- 不要寫入 `/home/ray/ros2_ws`。
- 不要重跑 Phase 7G/AE 已用盡的 runtime slots。
- 不要刪除 `ros2_evidence` 或覆寫唯一證據。
- 不要未經盤點就改 production controller、world、robot、map、route、AMCL 或 frozen docs。
- 實車最好獨立 branch/package/repo；先確認使用者是否要把 `/home/ray/ros2_racecar` 當模擬 reference。
- 車型未知，不能預設 differential 或 Ackermann。
