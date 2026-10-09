# WRO 2026 — Current State

更新時間：2026-07-22

## 權威狀態

- Phase 6J：**Done — J1 FINAL_ACCEPTANCE_PASS；FINAL ACCEPTED**。
- 正式 repo：`/home/ray/ros2_racecar`
- 執行 workspace：`/home/ray/ros2_ws`
- Production controller：`src/wro2026_sim/scripts/line_follower.py`
- Production controller SHA-256：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`
- 正式 controller 與 Phase 6H 權威 `semantic_candidate.py` byte-identical；Phase 6I runtime 使用的 diagnostic controller 與 runner hooks 已完整移除。
- formal、workspace、installed 三份 controller SHA 相同且兩組 `cmp=0`。
- Python syntax、`git diff --check` 與 package build 均通過。
- 本輪沒有修改 world、model、launch、NAT、DNS 或 Windows 設定。
- Phase 6J 最終驗收未修改 controller 或 build artifact；formal、workspace、installed 全程維持上述 production SHA。

## 正式保留的 production 行為

- Phase 6I：`OLD_EPOCH_ODOM_QUARANTINE`
  - Phase 5X time-rewind reset candidate 以 Phase 6H 凍結版本一併正式保留，77-field reset scope 與 rewind predicate 未修改。
  - reset-detecting odom callback 執行 reset、publish STOP 並立即 return，不再用觸發 reset 的舊 odom 重建 cache、pose 或 target。
  - reset barrier 期間拒絕 missing-header 或 header 晚於 node time 的 old-epoch odom；被拒絕 callback 只 STOP 且不更新 cache、pose、target 或 command。
  - 第一筆合法 fresh odom 才解除 barrier、由 fresh pose 建立 `_straight_target_yaw`；barrier 期間的非零 command request 強制轉為 STOP。
- Phase 5R：`BOTTOM_DIRECT_REORIENT_ODOM_HANDOFF`
  - 只作用於 bottom-context RECOVER；front-clear ordering 與 `0.45 m` threshold 維持不變。
  - bottom elapsed 到達 `5.000 s` 且 front 仍 non-clear 時，每個 bottom attempt 最多啟動一次，並保存原 target yaw。
  - DIRECT REORIENT：entry callback 不發 STOP、不等待 settle，直接 command `(linear=0, angular=+0.600)`，target `+pi/4`，heading predicate 正常完成，fail-safe `3.490 s`。
  - DIAGONAL REVERSE：沿用既有 bottom reverse command與 `+pi/4` target，以 cumulative planar odom path `0.027969393301043588 m` 正常完成，fail-safe `2.350 s`。
  - RESTORE：回復 entry 保存的 target yaw，command `(linear=0, |angular|=0.600)` 且方向依 heading error，fail-safe `3.505 s`。
  - route hard fail-safe `8.380 s`；正常完成後直接進 `POST_TURN`，不依賴 front scan、不進 generic RECOVER forward、不可重入。
  - 任一 segment 或 route fail-safe 耗盡時先 publish stop，再進既有 `RECOVER_FAILED`。
- Phase 5D：`RECOVER_EGRESS_EXTENSION`
  - 第一次 upper-PASS RECOVER front-clear 後啟動。
  - extension path `0.107710342260001 m`、timeout `2.730 s`，只由 odom path progress 正常完成。
  - ALIGN_DOWN target `-pi/2`、command `(0,+0.600)`、timeout `3.651 s`。
  - route hard fail-safe `5.896 s`，timeout-first／hard-first。
- Phase 3P：PASS elapsed cap 固定 `0.500 s`。
- Phase 3O：upper PASS RECOVER reverse-hold，front clear 前不進 forward。
- Phase 3H：POST_TURN pre-transition pending-set 修正。
- Phase 2U：bounded camera latch。
- Phase 2V：commanded-direction signed cumulative yaw progress。
- Phase 2O：bottom-context RECOVER 禁止 forward-turn。
- Phase 2W：bottom RECOVER entry target reset。

`BOTTOM_POST_TIMEOUT_SCAN_CONFIRM_4` 已移除；Phase 4Z POST_TURN bottom guard仍未保留。

## Phase 5Q Level 1 screening

- 唯一 candidate：`BOTTOM_DIRECT_REORIENT_ODOM_HANDOFF`。
- 分類：**Q1 CANDIDATE_SURVIVES_LEVEL1**；temporary controller SHA `12b988ea69d33b60ecda2f4978a01387891c9f1e17bbd6706f42cb038f3679a6`。
- 6/6 有效 B runs 完整 route 成功，5 次 fallback activation 全部依凍結順序正常完成；另 1 run 在 entry 前 front clear，activation 為 0。
- Level 1 後已完整 rollback，Phase 5R 只重新套用該保存 patch，沒有重新設計或調參。
- Evidence：`/home/ray/ros2_evidence/phase5q_bottom_direct_reorient_odom_handoff_screening_20260718_140225`

## Phase 5R Level 2 retention

- 分類：**R1 RETAINED**。
- No-obstacle：`2/2` 有效完成；fallback、extension、RECOVER_FAILED、timeout、exception、state loop 全部為 `0`。
- Production B：`8/8` 有效完整 route 成功；正常有效 rerun `0`，setup/protocol failure `0`。
- fallback activation `7/8`；`R_B05` 在第一個可處理的 post-5 callback 已 front clear，activation 正確為 `0`，仍完成完整 route。
- 7 次 activation 的 DIRECT／REVERSE／RESTORE 全部由正常 predicate 完成，順序正確，無 STOP dwell、重入、generic RECOVER forward、RECOVER_FAILED、timeout、exception 或 state loop。
- DIRECT duration `2.625–2.875 s`；最大實際 angular velocity `0.6065385893151476 rad/s`；yaw overshoot 全部 `0`。
- REVERSE duration `0.755–0.885 s`；formal cumulative odom path `0.02840635200706614–0.03299457012642676 m`。
- RESTORE duration `1.610–1.640 s`；yaw overshoot 全部 `0`。
- candidate route duration `4.990–5.375 s`，相對 `8.380 s` hard fail-safe 最小 margin `3.005 s`。
- candidate route 最低 clearance `137.892655 mm`；handoff 後 `2.0 s` 最低 clearance `160.457350 mm`。
- B 全程最低 clearance `38.073101 mm`（`R_B03`，`base_collision:obstacle_yellow_mid`）。
- `RECOVER_EGRESS_EXTENSION` 在 B route `8/8` bit-exact，route duration `5.120–5.375 s`。
- formal／workspace／installed 同步、syntax、diff check、build與 cleanup 全部通過。
- Level 2 evidence：`/home/ray/ros2_evidence/phase5r_direct_reorient_level2_retention_20260719_141424`
- Read-only archive：`/home/ray/ros2_evidence/phase5r_direct_reorient_level2_retention_20260719_141424.tar.gz`，SHA-256 `69729a5581502df4aa7ba86e730bbaf61a12b67055a06eb08f1f90bd3b8b7f6c`；獨立 GNU tar 解壓 verification 通過。

## Phase 6F root cause

- reset race 的權威根因是 reset-detecting odom callback 在 time rewind reset 後仍繼續處理觸發 reset 的 old-epoch odom，因而可用 reset 前 pose 重建 cache／target，並讓第一筆非零 command 使用 stale state。
- Phase 6F bounded campaign 與保存的 stale record 提供可重播的反例；本輪未重掃其歷史 raw evidence。

## Phase 6H Level 1 screening

- 分類：**H1 OLD_EPOCH_ODOM_QUARANTINE_LEVEL1_PASS**。
- 權威 semantic candidate SHA：`de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2`；runtime diagnostic SHA：`89cecb19e694736e9ad988dd7124bb6b37d0093e95160589d2c0d6b29e7dd32d`。
- 4/4 groups、12/12 valid slots；8/8 no-obstacle 與 4/4 B01 通過，正常有效 rerun `0`，stale/cache/barrier/control/safety errors 全部 `0`。
- Evidence：`/home/ray/ros2_evidence/phase6h_setup_repair_odom_quarantine_20260722_001814`；manifest SHA `58520aa6b2218f3cfd6909742cda02a1aab560dd69bf730a305659fa3e20dbe9`。

## Phase 6I Level 2 retention

- 分類：**I1 OLD_EPOCH_ODOM_QUARANTINE_LEVEL2_RETAINED**。
- Runtime 前完整 Phase 6H offline gate 全部 PASS；runtime diagnostic candidate 與 Phase 6H 使用版本 byte-identical。
- 2/2 cold-launch groups、8/8 valid slots，固定順序與單 PID contract 通過；I_G1 PID `6836`，I_G2 PID `7877`，正常有效 rerun `0`。
- I_G1 首次 launch 因 shell 未 source workspace 而在 slot 前 setup failure；未形成有效 slot，失敗紀錄保存後使用唯一允許 retry。其後未再重跑。
- 4/4 no-obstacle 抵達 `NO_OBSTACLE_BOTTOM_POSE`；4/4 B 抵達 `FIRST_BLUE_ACCEPT_PLUS_0P5`。
- 6 次 reset 中 4 次由 odom callback 偵測；4/4 immediate return、empty reset cache、fresh odom acceptance、fresh target 與 first nonzero command ordering 全通過。Runtime 沒有實際排隊 old-epoch odom；offline replay 的 multiple-old-header／missing-header rejection 全通過。
- Barrier nonzero publish、stale cache/target leak、contact、clearance `<10 mm`、exception、state loop與控制退化全為 `0`；全組最低 clearance `52.847743 mm`。
- Semantic production startup/topic smoke 的 controller 與 `/clock`、`/odom`、`/scan`、`/camera/image_raw`、`/cmd_vel` probes 全通過；停止時的同步 SIGINT traceback/exit `-2` 是 bounded smoke cleanup artifact，並非運行期 controller exception，未重跑 smoke。
- formal／workspace／installed SHA 一致、兩組 `cmp=0`；syntax、final build、`git diff --check` 與 cleanup 全部通過。
- Evidence：`/home/ray/ros2_evidence/phase6i_old_epoch_odom_quarantine_level2_retention_20260722_214929`。

## Phase 6J Final Production Acceptance

- 分類：**J1 FINAL_ACCEPTANCE_PASS；FINAL ACCEPTED**。
- 固定單一 cold-launch sequence：`NO01→reset→B01→reset→NO02→reset→B02`；`4/4` valid，controller PID 全程 `22996`，正常有效 rerun `0`，setup retry `0`。
- `2/2` no-obstacle 抵達 `NO_OBSTACLE_BOTTOM_POSE`；`2/2` B 抵達 `FORMAL_FULL_ROUTE_SUCCESS_PLUS_0P5`。
- 三次 reset 全部 clock rewind、start pose、readiness 與 first-command ordering 通過，old-epoch odom `0`。
- 一次 reset 由 odom callback 偵測；STOP／immediate return、empty cache、fresh odom acceptance/repopulation、fresh pose target與 barrier期間非零 command `0` 全部通過。另外兩次為 non-odom-detected reset，外部被動 probe 仍確認第一筆 start odom先於第一筆非零 command。
- contact、clearance `<10 mm`、RECOVER_FAILED、exception、unexpected state loop與generic RECOVER forward全部為 `0`；全組最低 clearance `54.794775 mm`。
- formal／workspace／installed SHA一致，兩組 `cmp=0`；syntax、`git diff --check`、shutdown與process cleanup全部通過。
- Evidence：`/home/ray/ros2_evidence/phase6j_final_production_acceptance_20260722_224955`。

## 已拒絕或關閉的方向

- Phase 4Z POST_TURN bottom guard。
- bottom timeout `5.420 s` 或任何大於 `5.000 s` 的固定 bottom timeout。
- Exact `REVERSE_EAST_STAGE`。
- A1/A2/A3 固定單段 PASS command。
- K1/K2/K3 固定 two-segment PASS trajectory。
- 放寬 camera/pose gate。
- side LiDAR guard。
- `BOTTOM_POST_TIMEOUT_SCAN_CONFIRM_4` 與 fixed scan-count bottom completion family。
- Phase 5H–5P 已正式分類並關閉的其他 candidate family；除非新證據直接推翻既有結論，不得重新開啟或變形重試。

## 歷史 Phase 限制

Phase 2A 到 Phase 6J 已完成或正式分類，不得重跑。不得重新掃描完整舊 evidence 或原始 logs；需要背景時只讀指定 summary。

## 唯一下一步

**FINAL ACCEPTED。Phase 6J 已完成且分類為 J1；不再建立後續 Phase。**
