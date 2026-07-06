# Next Prompt

Updated: 2026-07-06

Paste this into VSCode Codex for the next coding/testing round.

```text
下一輪任務：用 LiDAR scan cluster gate 區分窄障礙物與場地幾何，修正 B 點太晚觸發。

目前狀態：
- 無障礙誤觸發已修掉：avoid_start_count=0。
- B 點障礙物 x=0.00 y=0.42 仍會觸發，但太晚：
  avoid_start x=+0.087 y=+0.434，距障礙只有 0.088m。
- 不能只要求 near_front，因為 B 的早期訊號主要在 diag_near。
- 但無障礙上方幾何也會 diag_near，所以要做 scan cluster gate。
- 不要上 2 顆/3 顆，不要 AI/Nav2/停車。
- 不要 commit，除非達到 Done 或至少有明確改善。

目標：
允許 B 點斜前方障礙較早觸發，同時保持無障礙 avoid_start_count=0。

請修改 src/wro2026_sim/scripts/line_follower.py：

A. 新增 scan cluster 分析函式
使用 LaserScan 原始 ranges，在前方/斜前方 ROI 找近距離 cluster。

建議 ROI：
- front-left 到 front-right，例如 300 deg 到 60 deg，跨 0 度
- 或分兩段：300~359, 0~60
- threshold 可先用 0.45m 或 0.50m

輸出：
- cluster_count
- nearest_cluster_min_range
- nearest_cluster_angle_center_deg
- nearest_cluster_width_deg
- nearest_cluster_point_count

B. 判斷窄障礙物
黃色障礙物是 0.06m box，在 LiDAR 上應該是窄 cluster。
場地牆/中央島通常會形成比較寬的連續 cluster 或邊界長段。

建議 gate：
- cluster_min_range < 0.45 或 0.50
- cluster_width_deg between 4 and 28 deg
- cluster_point_count between 2 and 40
- cluster angle 在前方/斜前方 ROI
- 若 cluster_width_deg > 35 或 point_count 很大，視為 wall/island geometry，不觸發

C. Trigger 邏輯
不要只靠 near_front。
改成：
- obstacle_zone
- not retrigger locked
- cluster_obstacle == true
- plus side_clear 或 not narrow_geometry
才觸發 AVOID。

允許 diag-only trigger，但必須 cluster_obstacle true。

D. Trigger debug log
在接近 trigger 區時 throttle 印：
- front / front_min / LF / RF / LS / RS
- cluster_count
- cluster_min_range
- cluster_angle_center
- cluster_width_deg
- cluster_point_count
- cluster_obstacle
- trigger yes/no
- pose

E. 測試流程
1. 無障礙 regression：
- 把障礙物移回中央暫存
- 跑上方通道
- avoid_start_count 必須 = 0
- 記錄 cluster debug，證明無障礙 diag_near 不觸發

2. B 點測試：
- obstacle_yellow_mid x=0.00 y=0.42
- 目標 avoid_start distance from obstacle 要 > 0.12m，越接近 0.16~0.25m 越好
- 必須觸發 AVOID
- 不要 RECOVER_FAILED
- 最少要離開上方區 y < 0.20
- 最好回到底邊

F. 回報證據
必須附：
- git status --short --branch
- git diff --stat
- build
- topic/probe
- 無障礙 avoid_start_count
- 無障礙 cluster debug 摘錄
- B 點 avoid_start pose
- B 點 dx/dy/dist to obstacle
- B 點 cluster debug 摘錄
- AVOID/RETURN 主要 exit log
- left_upper_pose / bottom_pose
- recover / recover_failed
- 結論 Done / Partial / Blocked

G. Done 標準
- 無障礙 avoid_start_count=0
- B 點較早觸發，距障礙 > 0.12m
- B 點沒有 RECOVER_FAILED
- B 點能離開上方區 y < 0.20
- 最好能回到底邊
```

