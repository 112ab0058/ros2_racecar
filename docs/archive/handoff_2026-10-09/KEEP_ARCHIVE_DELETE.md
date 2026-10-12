> 已封存（2026-10-13）。目前狀態見 docs/STATUS.md。

# 保留、封存、候選清理：安全政策

## 必須保留
- `.git/`、目前 branch、未提交與未追蹤檔案。
- `docs/SPEC.md`, `CURRENT_STATE.md`, `TEST_EVIDENCE.md`, `NEXT_PROMPT.md`。
- 正式 `src/`、launch/config、URDF/Xacro、world/maps、package metadata、測試與 README。
- `/home/ray/ros2_evidence/` 下的 phase summaries、classification、manifests、hash、有效/失敗 runtime evidence。
- AE/AE-S/7G 的 datasets、frozen search outputs 與 campaign reports：它們是負結果的證據，不是垃圾。
- 未來實車的原始 rosbag、串口 logs、照片、電源測試、量測表與 calibration records。

## 封存但不直接部署
- Gazebo-only controller/observer/runner、world、simulation TF/odom instrumentation。
- AE wheel geometry fitting/search outputs、7G Generation B dataset/model/CV outputs。
- 模擬 map：只供參考，不能直接當實車 map。
- 歷史 phase prompt、candidate spec、phase temporary code 的清除紀錄。
這些是研究參考，不是可直接部署到 Jetson 的程式。

## 只能在盤點後列為清理候選
- repo-local `build/`, `install/`, `log/`（可重建，但先檢查是否有未追蹤內容與必要輸出）。
- duplicate exports、cache、暫存影片。
- 大型 rosbag/video/cache：只有確認有獨立完整備份、manifest 驗證通過且使用者同意後才可刪。
- `.phase*_tmp` 只能在確認不含使用者工作、phase rollback 也支持後處理。

## 絕對不要自行刪除/修改
- `/home/ray/ros2_ws`（歷史規則禁止寫入）。
- `.git`、使用者工作樹、未追蹤檔案、唯一 evidence。
- production controller/world/robot/map/route/AMCL。
- frozen docs 未經確認不得覆寫。
- Jetson 的 bootloader、device tree、電源/風扇/網路設定；未備份前不動。

## 清理流程
只讀 inventory → KEEP/ARCHIVE/CLEANUP/UNKNOWN 分類 → 獨立備份 → 驗證 SHA manifest → 使用者核准精確刪除清單 → 清理 → 保存 before/after report。不要直接 `rm -rf`。
